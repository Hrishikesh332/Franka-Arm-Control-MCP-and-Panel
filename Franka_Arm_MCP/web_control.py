"""Typed Desk web control, using the endpoints served by this FR3.

No browser automation, arbitrary HTTP tool, direct FCI loop or safety resets.
"""
from __future__ import annotations

import base64
import copy
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import tempfile
import threading
import time
from urllib.parse import urlencode

ROOT = Path(__file__).resolve().parent
SESSION = ROOT / ".session.json"
POLICY = ROOT / "task-policy.json"
ACTION_LOCK = threading.Lock()
SEND_LOCK = threading.Lock()
STOPPED = threading.Event()
FIXED = frozenset({
    ("POST", "/admin/api/login"),
    ("POST", "/admin/api/control-token/request"),
    ("DELETE", "/admin/api/control-token"),
    ("GET", "/desk/api/skills"),
    ("GET", "/desk/api/groups"),
    ("GET", "/desk/api/timelines"),
    ("POST", "/desk/api/timelines"),
    ("PUT", "/desk/api/timelines/0_move2dest2"),
    ("GET", "/desk/api/robot/configuration"),
    ("GET", "/desk/api/robot/status"),
    ("GET", "/admin/api/end-effector/profiles/active"),
    ("GET", "/desk/api/execution"),
    ("POST", "/desk/api/execution"),
    ("DELETE", "/desk/api/execution"),
    ("POST", "/desk/api/joints/lock"),
    ("POST", "/desk/api/joints/unlock"),
})


def write_private(path, data):
    descriptor, temporary = tempfile.mkstemp(prefix=".franka-", dir=ROOT)
    try:
        os.fchmod(descriptor, 0o600)
        with os.fdopen(descriptor, "w") as output:
            json.dump(data, output, indent=2)
            output.write("\n")
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def task_id(value):
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,256}", value):
        raise ValueError("Use an exact task ID from franka_real_list_tasks")
    return value


def task_digest(data):
    return hashlib.sha256(json.dumps(data, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


class DeskWebClient:
    def __init__(self, reader):
        self.reader = reader

    def session(self):
        if not SESSION.exists():
            raise RuntimeError("Desk login required. Run franka_hardware_mcp/Connect Franka.command locally; do not send your password in chat")
        if SESSION.stat().st_mode & 0o077:
            raise RuntimeError("Desk session file must have owner-only permissions (chmod 600)")
        data = json.loads(SESSION.read_text())
        if data.get("host") != self.reader.host or data.get("certificate_sha256") != self.reader.pin:
            raise RuntimeError("Desk session belongs to a different robot or certificate")
        token = data.get("authorization", "")
        if not re.fullmatch(r"[A-Za-z0-9_.-]+", token):
            raise RuntimeError("Invalid local Desk session; log in again")
        if data.get("expires_at", 0) <= time.time():
            raise RuntimeError("Desk login expired; run Connect Franka.command again")
        return data

    def request(self, method, path, body=None, *, form=False, login=False, control=False, start=False):
        timeline = r"/desk/api/timelines/[A-Za-z0-9_-]{1,256}"
        element = timeline + r"/\d{1,3}/\d{1,3}"
        permitted = ((method, path) in FIXED
                     or (method == "GET" and re.fullmatch(timeline, path))
                     or (method in {"POST", "PUT"} and re.fullmatch(element, path)))
        if not permitted:
            raise ValueError("Endpoint is outside the dedicated Franka control contract")
        headers = {"Accept": "application/json", "Content-Type": "application/x-www-form-urlencoded" if form else "application/json"}
        if not login:
            session = self.session()
            headers["Cookie"] = "authorization=" + session["authorization"]
            if control:
                token = session.get("control_token")
                if not token or not isinstance(token, str) or "\r" in token or "\n" in token:
                    raise RuntimeError("Take robot control with franka_real_take_control first")
                headers["X-Control-Token"] = token
        payload = None if body is None else (urlencode(body).encode() if form else json.dumps(body, allow_nan=False).encode())
        connection = self.reader.pinned_connection()
        sent = False
        try:
            # A stop and a task start have one send ordering. No queued movement
            # is replayed after stop. Socket writes retain the transport timeout.
            with SEND_LOCK:
                if start and STOPPED.is_set():
                    raise RuntimeError("Task execution is stopped; explicitly resume before a new task")
                connection.request(method, path, body=payload, headers=headers)
                sent = True
            response = connection.getresponse()
            raw = response.read(1048577)
            if len(raw) > 1048576:
                raise RuntimeError("Desk control response exceeded its size limit")
            if response.status not in (200, 201, 204):
                detail = ''
                if path.startswith('/desk/api/timelines'):
                    try:
                        error = json.loads(raw)
                        detail = ' ' + (json.dumps({key: error[key] for key in ('code', 'error', 'message', 'details') if key in error}) if isinstance(error, dict) else str(error))[:800]
                    except (ValueError, TypeError):
                        detail = ' ' + raw.decode(errors='replace')[:800]
                raise RuntimeError(f"Desk {method} {path}: HTTP {response.status}.{detail} Control was not confirmed; inspect current status before retrying")
            if not raw:
                return {"http_status": response.status}
            if login:
                return raw.decode().strip().strip('"')
            try:
                return json.loads(raw)
            except (ValueError, UnicodeError):
                raise RuntimeError("Unexpected response from the installed Desk API") from None
        except OSError:
            phase = "after sending the request; its outcome is unknown" if sent else "before sending the request"
            raise RuntimeError(f"Robot connection failed {phase}. Do not automatically retry a control action") from None
        finally:
            connection.close()

    def login(self, username, password):
        # Exact password transform used by this FR3's served Desk client.
        encoded = bytes((ord(char) & 255) for char in f"{password}#{username}@franka")
        digest = hashlib.sha256(encoded).digest()
        hashed = base64.b64encode(",".join(str(value) for value in digest).encode()).decode()
        token = self.request("POST", "/admin/api/login", {"login": username, "password": hashed}, login=True)
        try:
            claims = json.loads(base64.urlsafe_b64decode(token.split(".")[1] + "=="))
            # Desk's web session payload may have no JWT exp field: the web
            # client uses its cookie lifetime. Limit such sessions locally to
            # one hour; the robot remains authoritative on every API request.
            expires = float(claims.get("exp", time.time() + 3600))
        except (ValueError, KeyError, IndexError, TypeError):
            raise RuntimeError("Robot did not return a usable Desk session") from None
        if expires <= time.time():
            raise RuntimeError("Robot session is already expired; check the robot and Mac clocks")
        write_private(SESSION, {"host": self.reader.host, "certificate_sha256": self.reader.pin, "authorization": token,
                                "username": username, "expires_at": expires})
        return {"authenticated": True, "username": username, "expires_at_utc": datetime.fromtimestamp(expires, timezone.utc).isoformat()}

    def list_tasks(self):
        data = self.request("GET", "/desk/api/timelines")
        if not isinstance(data, list):
            raise RuntimeError("Unexpected Desk timeline list format")
        return {"source": "physical_franka_desk_web_api", "tasks": [
            {key: task[key] for key in ("id", "name", "valid", "description") if key in task}
            for task in data if isinstance(task, dict)
        ]}

    def robot_state(self):
        started = datetime.now(timezone.utc).isoformat()
        configuration = self.request("GET", "/desk/api/robot/configuration")
        status = self.request("GET", "/desk/api/robot/status")
        profile = self.request("GET", "/admin/api/end-effector/profiles/active")
        return {"source": "physical_franka_desk_web_api", "read_only": True,
                "read_started_at_utc": started, "read_finished_at_utc": datetime.now(timezone.utc).isoformat(),
                "snapshot_is_atomic": False, "configuration": configuration, "robot_status": status,
                "configured_end_effector_profile": profile, "pose_layout": "column-major 4x4, translation at indices 12,13,14 in metres",
                "table_geometry_available": False,
                "note": "Configured tool data is not a measurement of the tool actually attached. Estimated forces are not a calibrated collision or grasp verdict."}

    def diagnostic_events(self, basic_authorization=None):
        """Read the documented public event log; never alter error/recovery state."""
        session = self.session()
        headers = {'Accept':'text/plain','Cookie':'authorization='+session['authorization']}
        if basic_authorization is not None:
            if not isinstance(basic_authorization,str) or not re.fullmatch(r'Basic [A-Za-z0-9+/]+=*',basic_authorization):
                raise ValueError('Invalid diagnostic Basic authentication header')
            headers['Authorization'] = basic_authorization
        connection = self.reader.pinned_connection()
        try:
            connection.request('GET','/api/system/log/public/events',headers=headers)
            response = connection.getresponse()
            if response.status!=200:
                raise RuntimeError(f'Event-log read returned HTTP {response.status}; no robot changes were sent')
            if not response.getheader('Content-Type','').lower().startswith('text/plain'):
                raise RuntimeError('Unexpected event-log content type')
            limit = 16*1024*1024
            raw = response.read(limit+1)
            if len(raw)>limit:
                raise RuntimeError('Event log exceeds 16 MiB; no truncated diagnosis will be reported')
        finally:
            connection.close()
        label = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        folder = ROOT/'diagnostics'/label
        folder.mkdir(parents=True,exist_ok=False)
        log = raw.decode('utf-8',errors='replace')
        fd = os.open(folder/'events.log',os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
        with os.fdopen(fd,'w') as output:
            output.write(log)
        relevant = [line for line in log.splitlines() if re.search(r'singular|inverse|kinemat|elbow|motion.generator|planning|exception',line,re.I)]
        result = {'read_only':True,'source':'physical_franka_documented_public_event_log',
                  'event_log_file':str(folder/'events.log'),'byte_count':len(raw),
                  'relevant_line_count':len(relevant),'recent_matching_lines':relevant[-40:],
                  'log_is_truncated':False,'motion_requested':False,'safety_reset_requested':False}
        write_private(folder/'read_report.json',result)
        return result

    def preview(self, ident):
        ident = task_id(ident)
        data = self.request("GET", "/desk/api/timelines/" + ident)
        approved = json.loads(POLICY.read_text()).get("approved_tasks", {}) if POLICY.exists() else {}
        digest = task_digest(data)
        return {"task_id": ident, "task_sha256": digest, "enabled_locally": approved.get(ident) == digest,
                "task": data, "motion_requested": False}

    def prepare_table_transfer(self):
        """Upload only the locally checked, explicitly requested v04 routine."""
        folder = ROOT / 'plans' / 'table_transfer_v04'
        payload = json.loads((folder / 'new_task.json').read_text())
        check = json.loads((folder / 'nominal_path_check.json').read_text())
        name = 'chat_bare_flange_x_transfer_slow_v04'
        if payload.get('name') != name or payload.get('id') != '0_move2dest' or check.get('task_name') != name:
            raise RuntimeError('Unexpected table-transfer artifact identity')
        if check.get('sample_count', 0) < 100 or check.get('minimum_sampled_flange_z_m', 0) < .22499:
            raise RuntimeError('Missing nominal path/clearance check')
        if payload.get('parameter', {}).get('speed') != .05:
            raise RuntimeError('Unexpected task speed')
        points = payload['parameter']['parameterized_poses']
        elements = payload['containers'][0]['elements']
        if len(points) != 10 or len(elements) != 10:
            raise RuntimeError('Unexpected waypoint count')
        for index, (point, element) in enumerate(zip(points, elements), 1):
            pose = point['pose_with_joint_angles']['pose']
            params = element['parameter']
            if len(pose) != 16 or pose[14] < .22499 or element.get('link') != 'cartesian_motion':
                raise RuntimeError('Unexpected waypoint or app')
            if any(params.get(key) != .05 for key in ('app_speed', 'app_acceleration', 'app_deceleration')):
                raise RuntimeError('Unexpected motion speed or acceleration')
            if params['motion'] != [{'access': ['parameter', 'parameterized_poses', index, 'pose_with_joint_angles']}]:
                raise RuntimeError('Unexpected waypoint reference')
        existing = self.list_tasks()['tasks']
        originals = {item['id']: self.preview(item['id'])['task_sha256'] for item in existing}
        matching = [item for item in existing if item.get('name') == name]
        if matching:
            raise RuntimeError('This task name already exists; inspect it instead of overwriting or uploading again')
        self.request('POST', '/desk/api/timelines', payload)
        after = self.list_tasks()['tasks']
        created = [item for item in after if item.get('name') == name]
        if len(created) != 1:
            raise RuntimeError('Upload outcome is not confirmed; inspect task list before retrying')
        for ident, digest in originals.items():
            if self.preview(ident)['task_sha256'] != digest:
                raise RuntimeError('An existing task differs after import; stop and inspect before any execution')
        remote = self.preview(created[0]['id'])
        actual = remote['task']
        for key in ('name', 'parameter', 'containers'):
            if actual.get(key) != payload.get(key):
                raise RuntimeError(f'Uploaded task {key} differs from the checked payload; execution remains disabled')
        result = {'task_id': created[0]['id'], 'task_name': name, 'task_sha256': remote['task_sha256'],
                  'server_reports_valid': actual.get('valid'), 'uploaded': True,
                  'enabled_for_execution': False, 'motion_requested': False}
        (folder / 'uploaded_task.json').write_text(json.dumps(actual, indent=2) + '\n')
        (folder / 'upload_verified.json').write_text(json.dumps(result, indent=2) + '\n')
        return result

    def set_workflow_speed(self, percent):
        if isinstance(percent, bool) or not isinstance(percent, (int, float)) or not math.isfinite(percent) or not 1 <= percent <= 40:
            raise ValueError('Workflow speed must be between 1 and the user-authorized maximum of 40 percent')
        with ACTION_LOCK:
            ident = '0_move2dest2'
            if self.request('GET', '/desk/api/execution').get('running') is not False:
                raise RuntimeError('Stop the active routine before changing its saved speed; no automatic restart is performed')
            before = self.preview(ident)
            if before['task'].get('name') != 'chat_bare_flange_x_transfer_slow_v04':
                raise RuntimeError('The configured task identity changed; no speed update was sent')
            if not before['enabled_locally']:
                raise RuntimeError('Task content changed since local enablement; inspect before updating speed')
            label = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
            folder = ROOT / 'plans' / 'table_transfer_v04' / ('speed_change_' + label)
            folder.mkdir(exist_ok=False)
            (folder/'before_task.json').write_text(json.dumps(before['task'], indent=2)+'\n')
            parameters = copy.deepcopy(before['task']['parameter'])
            parameters['speed'] = float(percent)/100
            self.request('PUT', '/desk/api/timelines/'+ident, {'parameter': json.dumps(parameters)}, form=True)
            after = self.preview(ident)
            expected = copy.deepcopy(before['task'])
            expected['parameter']['speed'] = float(percent)/100
            if after['task'] != expected:
                raise RuntimeError('Task read-back differs from the expected speed-only update; execution remains unverified')
            policy = json.loads(POLICY.read_text())
            policy['approved_tasks'][ident] = after['task_sha256']
            policy['last_speed_change_percent'] = percent
            write_private(POLICY, policy)
            result = {'task_id':ident, 'task_name':after['task']['name'], 'workflow_speed_percent':percent,
                      'previous_workflow_speed_percent':before['task']['parameter']['speed']*100,
                      'per_motion_speed_settings_changed':False, 'waypoints_changed':False,
                      'verified_by_robot_read_back':True, 'automatic_restart':False,
                      'task_sha256':after['task_sha256']}
            (folder/'after_task.json').write_text(json.dumps(after['task'], indent=2)+'\n')
            (folder/'change_report.json').write_text(json.dumps(result, indent=2)+'\n')
            return result

    def prepare_ground_relative_transfer(self):
        return self._prepare_ground_revision('ground_relative_v01','chat_ground_relative_return_v01',15,.4,.15,.05)

    def prepare_ground_y_transfer(self):
        return self._prepare_ground_revision('ground_relative_v02','chat_ground_y100_x_return_v02',18,.7,.1,.01)

    def prepare_no_rotation_transfer(self):
        return self._prepare_ground_revision('ground_relative_v03','chat_ground_no_rotation_speed50_v03',10,.7,.1,.01,.5)

    def prepare_no_rotation_speed30(self):
        return self._prepare_ground_revision('ground_relative_v04','chat_ground_no_rotation_speed30_v04',10,.7,.1,.01,.3)

    def prepare_backwards_x_first(self):
        return self._prepare_ground_revision('ground_relative_v05','chat_backwards_x_first_speed30_v05',10,.7,.1,.01,.3)

    def prepare_home_tasks(self):
        """Import only the two authored, checked Home/Ready tasks; never move."""
        with ACTION_LOCK:
            folder = ROOT/'plans/home_setup_v02'
            manifest = json.loads((folder/'manifest.json').read_text())
            home = json.loads((folder/'home_task.json').read_text())
            workflow = json.loads((folder/'workflow_task.json').read_text())
            if (home.get('name')!='Home_Franka_Ready_30_v01' or
                workflow.get('name')!='chat_backward_x_from_franka_ready_v06' or
                task_digest(home)!=manifest.get('home_payload_sha256') or
                task_digest(workflow)!=manifest.get('workflow_payload_sha256')):
                raise RuntimeError('Home task artifact identity/hash changed')
            if (not manifest.get('nominal_checks_passed') or
                manifest.get('new_nominal_joint_transition_sample_count',0)<200 or
                manifest.get('minimum_transition_virtual_tip_reference_gap_m',0)<.01):
                raise RuntimeError('Nominal Home-to-workcell transition check is missing')
            canonical = [0,-math.pi/4,0,-3*math.pi/4,0,math.pi/2,math.pi/4]
            target = manifest['home_pose']
            if (target['joint_angles']!=canonical or
                home['parameter']['parameterized_poses'][0]['pose_with_joint_angles']!=target or
                workflow['parameter']['parameterized_poses'][-1]['pose_with_joint_angles']!=target):
                raise RuntimeError('The selected standard Ready Home target changed')
            if home['parameter']['speed']!=.3 or workflow['parameter']['speed']!=.7:
                raise RuntimeError('Home or workflow slider setting changed')
            for payload,expected_count in ((home,1),(workflow,12)):
                apps = payload['containers'][0]['elements']
                points = payload['parameter']['parameterized_poses']
                if len(apps)!=expected_count or len(points)!=expected_count:
                    raise RuntimeError('Home/workflow waypoint count changed')
                for index,app in enumerate(apps,1):
                    p = app['parameter']
                    if p.get('app_speed')!=.3 or p.get('app_acceleration')!=.05:
                        raise RuntimeError('A Home/workflow component is not at 30 percent speed')
                    if p.get('motion')!=[{'access':['parameter','parameterized_poses',index,'pose_with_joint_angles']}]:
                        raise RuntimeError('Home/workflow pose reference changed')
                    if app.get('link') not in ('joint_motion','cartesian_motion'):
                        raise RuntimeError('Unexpected Home/workflow motion app')
            if (workflow['parameter']['parameterized_poses'][0]['name']!='Start at HOME Ready: J=[0,-45,0,-135,0,90,45] deg' or
                workflow['parameter']['parameterized_poses'][-1]['name']!='Return to HOME Ready [0,-45,0,-135,0,90,45] deg' or
                'HOME Ready' not in workflow['containers'][0]['elements'][0]['name'] or
                'HOME Ready' not in workflow['containers'][0]['elements'][-1]['name']):
                raise RuntimeError('The persisted in-task Home instructions are missing')
            if (home['containers'][0]['elements'][0]['link']!='joint_motion' or
                workflow['containers'][0]['elements'][0]['link']!='joint_motion' or
                workflow['containers'][0]['elements'][-1]['link']!='joint_motion'):
                raise RuntimeError('Home transitions must use the installed native Joint Motion app')
            for ident,expected in manifest['source_task_hashes'].items():
                if self.preview(ident)['task_sha256']!=expected:
                    raise RuntimeError('A taught source/start or backward-X task changed before Home setup')
            original = self.preview('0_move2dest7')['task']
            expected_points = ([{'name':manifest['entry_label'],
                                'pose_with_joint_angles':manifest['intermediate_workcell_start']}]+
                               original['parameter']['parameterized_poses']+
                               [{'name':manifest['finish_label'],'pose_with_joint_angles':target}])
            if workflow['parameter']['parameterized_poses']!=expected_points:
                raise RuntimeError('The preserved Cartesian backward-X route changed')
            for index,(app,old_app) in enumerate(zip(workflow['containers'][0]['elements'][1:-1],original['containers'][0]['elements']),2):
                expected_app = copy.deepcopy(old_app)
                expected_app['id'] = f'cartesian_motion_{index}'
                expected_app['parameter']['motion'] = [{'access':['parameter','parameterized_poses',index,'pose_with_joint_angles']}]
                if app!=expected_app:
                    raise RuntimeError('A preserved backward-X Cartesian app changed')
            for payload in (home,workflow):
                if any(payload['parameter'][key]!=original['parameter'][key] for key in ('safety','joint_impedance','cartesian_impedance')):
                    raise RuntimeError('Safety or impedance settings changed during Home setup')
            existing = self.list_tasks()['tasks']
            originals = {item['id']:self.preview(item['id'])['task_sha256'] for item in existing}
            results = []
            for filename,payload in (('home',home),('workflow',workflow)):
                matches = [t for t in self.list_tasks()['tasks'] if t.get('name')==payload['name']]
                if not matches:
                    self.request('POST','/desk/api/timelines',payload)
                    matches = [t for t in self.list_tasks()['tasks'] if t.get('name')==payload['name']]
                if len(matches)!=1:
                    raise RuntimeError('Home task import outcome is ambiguous; inspect before retrying')
                remote = self.preview(matches[0]['id'])
                if remote['task'].get('valid') is not True:
                    raise RuntimeError('Desk reports the imported Home/workflow invalid; inspect it before use')
                for key in ('name','parameter','containers','contextMenu'):
                    if remote['task'].get(key)!=payload.get(key):
                        raise RuntimeError('Robot Home/workflow read-back differs from authored artifacts')
                result = {'task_id':matches[0]['id'],'task_name':payload['name'],
                          'task_sha256':remote['task_sha256'],'server_reports_valid':remote['task'].get('valid'),
                          'enabled_for_execution':False,'motion_requested':False}
                (folder/(filename+'_uploaded_task.json')).write_text(json.dumps(remote['task'],indent=2)+'\n')
                (folder/(filename+'_upload_verified.json')).write_text(json.dumps(result,indent=2)+'\n')
                results.append(result)
            for ident,expected in originals.items():
                if self.preview(ident)['task_sha256']!=expected:
                    raise RuntimeError('An existing task changed during Home import')
            result = {'tasks':results,'existing_tasks_unchanged':True,'motion_requested':False,
                      'home_joint_angles_deg':manifest['home_joint_angles_deg'],
                      'home_approach_from_current_pose_checked':False}
            (folder/'upload_verified.json').write_text(json.dumps(result,indent=2)+'\n')
            return result

    def prepare_flange_inward_test(self):
        """Save only the checked 10 mm local -Z / Home return test."""
        with ACTION_LOCK:
            folder = ROOT/'plans/flange_inward_10mm_v01'
            task = json.loads((folder/'new_task.json').read_text())
            check = json.loads((folder/'manifest.json').read_text())
            if task.get('name')!='chat_flange_local_minus_z_10mm_30_v01' or task_digest(task)!=check.get('task_payload_sha256'):
                raise RuntimeError('Flange test identity/hash changed')
            if not check.get('nominal_checks_passed') or check.get('sample_count',0)<40 or check.get('minimum_virtual_tip_reference_gap_m',0)<.01:
                raise RuntimeError('Flange test nominal clearance check is missing')
            ready = check['start_return_pose']
            canonical = [0,-math.pi/4,0,-3*math.pi/4,0,math.pi/2,math.pi/4]
            points = task['parameter']['parameterized_poses']
            apps = task['containers'][0]['elements']
            if ready['joint_angles']!=canonical or len(points)!=2 or len(apps)!=2 or task['parameter']['speed']!=.3:
                raise RuntimeError('The flange test must start/return at standard Ready with two motions at 30 percent')
            target = points[0]['pose_with_joint_angles']['pose']
            home = ready['pose']
            if (any(abs(target[i]-home[i])>1e-10 for i in range(12)) or
                any(abs(target[12+i]-(home[12+i]-.01*home[8+i]))>1e-10 for i in range(3))):
                raise RuntimeError('Target must be exactly 10 mm along flange-local negative Z with fixed orientation')
            final = points[-1]['pose_with_joint_angles']
            if (max(abs(a-b) for a,b in zip(final['pose'],home))>1e-10 or
                max(abs(a-b) for a,b in zip(final['joint_angles'],canonical))>1e-4):
                raise RuntimeError('Flange test must return to the complete Ready Home configuration')
            for i,app in enumerate(apps,1):
                p = app['parameter']
                if app.get('link')!='cartesian_motion' or p.get('app_speed')!=.3 or any(p.get(k)!=.05 for k in ('app_acceleration','app_deceleration')):
                    raise RuntimeError('Flange test motion settings changed')
                if (p['motion']!=[{'access':['parameter','parameterized_poses',i,'pose_with_joint_angles']}] or
                    len(p['motion_params'])!=1 or p['motion_params'][0].get('point2point') is not False or
                    p['motion_params'][0].get('custom_speed')!=.3):
                    raise RuntimeError('Flange test Cartesian pose reference or speed changed')
            for ident,digest in check['source_task_hashes'].items():
                if self.preview(ident)['task_sha256']!=digest:
                    raise RuntimeError('A flange test Home/template/Ground reference changed')
            template = self.preview('0_move2dest8')['task']
            if any(task['parameter'][k]!=template['parameter'][k] for k in ('safety','joint_impedance','cartesian_impedance')):
                raise RuntimeError('Flange test safety or impedance settings changed')
            original = {t['id']:self.preview(t['id'])['task_sha256'] for t in self.list_tasks()['tasks']}
            matches = [t for t in self.list_tasks()['tasks'] if t.get('name')==task['name']]
            if not matches:
                self.request('POST','/desk/api/timelines',task)
                matches = [t for t in self.list_tasks()['tasks'] if t.get('name')==task['name']]
            if len(matches)!=1:
                raise RuntimeError('Flange task import outcome is ambiguous; inspect before retrying')
            remote = self.preview(matches[0]['id'])
            if remote['task'].get('valid') is not True or any(remote['task'].get(k)!=task.get(k) for k in ('name','parameter','containers')):
                raise RuntimeError('Flange task read-back does not match the checked plan')
            for ident,digest in original.items():
                if self.preview(ident)['task_sha256']!=digest:
                    raise RuntimeError('An existing task changed during flange-test import')
            result = {'task_id':matches[0]['id'],'task_name':task['name'],'task_sha256':remote['task_sha256'],
                      'server_reports_valid':True,'existing_tasks_unchanged':True,
                      'enabled_for_execution':False,'motion_requested':False}
            (folder/'uploaded_task.json').write_text(json.dumps(remote['task'],indent=2)+'\n')
            (folder/'upload_verified.json').write_text(json.dumps(result,indent=2)+'\n')
            return result

    def _prepare_ground_revision(self, revision, name, count, speed, tool_length, gap, app_speed=.05):
        folder = ROOT / 'plans' / revision
        payload = json.loads((folder/'new_task.json').read_text())
        check = json.loads((folder/'plan_check.json').read_text())
        if payload.get('name') != name or task_digest(payload) != check.get('task_payload_sha256'):
            raise RuntimeError('Ground-relative task artifact identity/hash changed')
        if not check.get('nominal_kinematics_passed') or check.get('sample_count',0)<300 or check.get('waypoint_count')!=count:
            raise RuntimeError('Ground-relative nominal path check is missing')
        if check.get('virtual_tool_length_m') != tool_length or check.get('minimum_sampled_virtual_tip_gap_m',0)<gap:
            raise RuntimeError('The specified virtual tool clearance has not been checked')
        if payload['parameter']['speed'] != speed:
            raise RuntimeError('Unexpected workflow speed')
        if revision in ('ground_relative_v02','ground_relative_v03','ground_relative_v04','ground_relative_v05'):
            independent = json.loads((folder/'independent_check.json').read_text())
            if (not independent.get('independent_nominal_checks_passed') or
                independent.get('task_payload_sha256')!=task_digest(payload) or
                independent.get('minimum_fk_virtual_tip_reference_gap_m',0)<gap):
                raise RuntimeError('Independent nominal path verification missing or stale')
        for ident, expected in check['source_task_hashes'].items():
            if self.preview(ident)['task_sha256'] != expected:
                raise RuntimeError('A taught Ground/source/destination task changed; regenerate this revision')
        reference = check['ground_reference']['near_surface_pose_z_m']
        points = payload['parameter']['parameterized_poses']
        elements = payload['containers'][0]['elements']
        if len(points)!=count or len(elements)!=count:
            raise RuntimeError('Unexpected ground-relative waypoint count')
        for index,(point,element) in enumerate(zip(points,elements),1):
            pose = point['pose_with_joint_angles']['pose']
            params = element['parameter']
            if len(pose)!=16 or pose[14]+pose[10]*tool_length-reference < gap-1e-6:
                raise RuntimeError('Waypoint violates the virtual tip clearance')
            if element.get('link')!='cartesian_motion' or params.get('app_speed')!=app_speed or any(params.get(key)!=.05 for key in ('app_acceleration','app_deceleration')):
                raise RuntimeError('Unexpected app or individual motion setting')
            if params['motion'] != [{'access':['parameter','parameterized_poses',index,'pose_with_joint_angles']}]:
                raise RuntimeError('Unexpected ground-relative pose reference')
        if revision == 'ground_relative_v02':
            poses = [p['pose_with_joint_angles']['pose'] for p in points]
            source, rotated, shifted, lifted, transferred, destination = [poses[i] for i in (2,6,7,8,9,10)]
            if (abs(shifted[13]-rotated[13]-.1)>1e-9 or
                any(abs(shifted[i]-rotated[i])>1e-9 for i in (12,14)) or
                any(abs(transferred[i]-lifted[i])>1e-9 for i in (13,14)) or
                transferred[12]>=lifted[12] or abs(lifted[14]-shifted[14]-.05)>1e-9 or
                abs(source[14]-destination[14])>1e-9 or
                abs(transferred[14]-destination[14]-.05)>1e-9):
                raise RuntimeError('Requested Y shift, X-only transfer or endpoint descent changed')
        if revision in ('ground_relative_v03','ground_relative_v04','ground_relative_v05'):
            if (not independent.get('orientation_constant') or not independent.get('joint3_constant') or
                independent.get('fixed_joint3_min_sigma',0)<=.01 or
                independent.get('fixed_joint3_max_condition',float('inf'))>=200):
                raise RuntimeError('No-rotation singularity screening missing or stale')
            poses = [p['pose_with_joint_angles']['pose'] for p in points]
            expected_start = check['start_return_pose']
            for point,element in zip(points,elements):
                pose_and_joints = point['pose_with_joint_angles']
                pose = pose_and_joints['pose']
                if (any(abs(pose[i]-expected_start['pose'][i])>1e-10 for i in range(12)) or
                    abs(pose_and_joints['joint_angles'][2]-expected_start['joint_angles'][2])>1e-10):
                    raise RuntimeError('No-rotation task changes orientation or the fixed elbow joint')
                motion_params = element['parameter']['motion_params']
                if len(motion_params)!=1 or motion_params[0].get('custom_speed')!=app_speed or motion_params[0].get('point2point') is not False:
                    raise RuntimeError('No-rotation component speed or Cartesian motion changed')
            if revision == 'ground_relative_v05':
                source, lifted, transferred, shifted, destination = [poses[i] for i in (2,3,4,5,6)]
                if (any(abs(lifted[i]-source[i])>1e-9 for i in (12,13)) or
                    abs(lifted[14]-source[14]-.05)>1e-9 or
                    any(abs(transferred[i]-lifted[i])>1e-9 for i in (13,14)) or
                    transferred[12]>=lifted[12] or
                    abs(shifted[13]-transferred[13]-.1)>1e-9 or
                    any(abs(shifted[i]-transferred[i])>1e-9 for i in (12,14)) or
                    any(abs(destination[i]-shifted[i])>1e-9 for i in (12,13)) or
                    abs(shifted[14]-destination[14]-.05)>1e-9 or
                    abs(source[14]-destination[14])>1e-9):
                    raise RuntimeError('Backward-X-first path, destination Y shift or endpoint descent changed')
            else:
                source, shifted, lifted, transferred, destination = [poses[i] for i in (2,3,4,5,6)]
                if (abs(shifted[13]-source[13]-.1)>1e-9 or
                    any(abs(shifted[i]-source[i])>1e-9 for i in (12,14)) or
                    any(abs(transferred[i]-lifted[i])>1e-9 for i in (13,14)) or
                    transferred[12]>=lifted[12] or abs(lifted[14]-shifted[14]-.05)>1e-9 or
                    abs(source[14]-destination[14])>1e-9 or
                    abs(transferred[14]-destination[14]-.05)>1e-9):
                    raise RuntimeError('No-rotation Y shift, X-only transfer or endpoint descent changed')
        if points[-1]['pose_with_joint_angles'] != check['start_return_pose']:
            # Numerical IK may differ at rounding level; compare position,
            # orientation and all joints rather than changing the checked plan.
            final = points[-1]['pose_with_joint_angles']
            expected = check['start_return_pose']
            if max(abs(a-b) for a,b in zip(final['pose'],expected['pose']))>1e-8 or max(abs(a-b) for a,b in zip(final['joint_angles'],expected['joint_angles']))>1e-4:
                raise RuntimeError('Final pose does not return to the captured start')
        existing = self.list_tasks()['tasks']
        if any(item.get('name')==name for item in existing):
            raise RuntimeError('This revision already exists; inspect it instead of overwriting')
        originals = {item['id']:self.preview(item['id'])['task_sha256'] for item in existing}
        self.request('POST','/desk/api/timelines',payload)
        created = [item for item in self.list_tasks()['tasks'] if item.get('name')==name]
        if len(created)!=1:
            raise RuntimeError('New-task upload outcome is unknown; inspect before retrying')
        for ident,expected in originals.items():
            if self.preview(ident)['task_sha256']!=expected:
                raise RuntimeError('An existing task changed during import; stop and inspect')
        remote = self.preview(created[0]['id'])
        for key in ('name','parameter','containers'):
            if remote['task'].get(key)!=payload.get(key):
                raise RuntimeError('Robot read-back differs from the checked ground-relative task')
        result = {'task_id':created[0]['id'],'task_name':name,'task_sha256':remote['task_sha256'],
                  'server_reports_valid':remote['task'].get('valid'),'existing_tasks_unchanged':True,
                  'uploaded':True,'enabled_for_execution':False,'motion_requested':False,
                  'workflow_speed_percent':speed*100,'per_motion_speed_percent':app_speed*100,
                  'required_start_xyz_m':check['start_return_pose']['pose'][12:15]}
        (folder/'uploaded_task.json').write_text(json.dumps(remote['task'],indent=2)+'\n')
        (folder/'upload_verified.json').write_text(json.dumps(result,indent=2)+'\n')
        return result

    def take_control(self):
        with ACTION_LOCK:
            existing = self.reader.get("/api/system/control-token")
            session = self.session()
            if session.get("control_id") is not None and existing.get("tokenId") == session["control_id"]:
                return {"control_owned": True, "owner": "franka-real", "already_owned": True}
            if existing.get("owner"):
                raise RuntimeError("Robot control is already held by " + str(existing["owner"]) + "; release it in Desk first. This MCP never forces takeover")
            result = self.request("POST", "/admin/api/control-token/request", {"requestedBy": "franka-real"})
            if not isinstance(result.get("token"), str) or result.get("id") is None:
                raise RuntimeError("Desk did not grant a usable control token")
            session.update(control_token=result["token"], control_id=result["id"])
            write_private(SESSION, session)
            return {"control_owned": True, "owner": "franka-real", "motion_requested": False}

    def release_control(self):
        with ACTION_LOCK:
            session = self.session()
            token = session.get("control_token")
            if not token:
                return {"control_released": True, "already_released": True}
            self.request("DELETE", "/admin/api/control-token", {"token": token}, control=True)
            session.pop("control_token", None)
            session.pop("control_id", None)
            write_private(SESSION, session)
            return {"control_released": True}

    def brakes(self, unlocked):
        with ACTION_LOCK:
            self.request("POST", "/desk/api/joints/" + ("unlock" if unlocked else "lock"), form=True, control=True)
            return {"joint_brakes": self.reader.get("/api/arm/joints")}

    def execute(self, ident, expected_sha256):
        with ACTION_LOCK:
            if STOPPED.is_set():
                raise RuntimeError("Task execution is stopped; explicitly resume before starting a new task")
            preview = self.preview(ident)
            if not preview["enabled_locally"] or preview["task_sha256"] != expected_sha256:
                raise RuntimeError("Task is not locally enabled with this exact content. Review and enable it through Enable Franka Tasks.command")
            name = preview['task'].get('name')
            if name == 'Home_Franka_Ready_30_v01':
                raise RuntimeError('Home target is saved as a preset; its approach from the current pose is not qualified for MCP execution')
            if name in ('chat_bare_flange_x_transfer_slow_v04','chat_ground_relative_return_v01','chat_ground_y100_x_return_v02','chat_ground_no_rotation_speed50_v03','chat_ground_no_rotation_speed30_v04','chat_backwards_x_first_speed30_v05','chat_backward_x_from_franka_ready_v06','chat_flange_local_minus_z_10mm_30_v01','Pick & Place Tomato v2 - Source Approach Fix'):
                observed = self.robot_state()
                actual = observed['configuration']['cartesianPose']
                if name == 'Pick & Place Tomato v2 - Source Approach Fix':
                    check = json.loads((ROOT/'plans/tomato_pick_place_v02/manifest.json').read_text())
                    if self.preview(check['source_task_id'])['task_sha256']!=check['source_task_sha256']:
                        raise RuntimeError('The original Tomato task changed since versioning; execution refused')
                    expected=check['start_return_pose']
                    start={'cartesianPose':expected['pose'],'jointAngles':expected['joint_angles']}
                elif name in ('chat_backward_x_from_franka_ready_v06','chat_flange_local_minus_z_10mm_30_v01'):
                    revision = 'flange_inward_10mm_v01' if name=='chat_flange_local_minus_z_10mm_30_v01' else 'home_setup_v02'
                    check = json.loads((ROOT/'plans'/revision/'manifest.json').read_text())
                    for source_id,digest in check['source_task_hashes'].items():
                        if self.preview(source_id)['task_sha256']!=digest:
                            raise RuntimeError('A source or Home setup task changed since planning; execution refused')
                    expected = check['start_return_pose']
                    start = {'cartesianPose':expected['pose'],'jointAngles':expected['joint_angles']}
                elif name in ('chat_ground_relative_return_v01','chat_ground_y100_x_return_v02','chat_ground_no_rotation_speed50_v03','chat_ground_no_rotation_speed30_v04','chat_backwards_x_first_speed30_v05'):
                    revision = {'chat_ground_relative_return_v01':'ground_relative_v01',
                                'chat_ground_y100_x_return_v02':'ground_relative_v02',
                                'chat_ground_no_rotation_speed50_v03':'ground_relative_v03',
                                'chat_ground_no_rotation_speed30_v04':'ground_relative_v04',
                                'chat_backwards_x_first_speed30_v05':'ground_relative_v05'}[name]
                    check = json.loads((ROOT/'plans'/revision/'plan_check.json').read_text())
                    for source_id,digest in check['source_task_hashes'].items():
                        if self.preview(source_id)['task_sha256']!=digest:
                            raise RuntimeError('Ground or source pose changed since planning; execution refused')
                    expected = check['start_return_pose']
                    start = {'cartesianPose':expected['pose'],'jointAngles':expected['joint_angles']}
                else:
                    start = json.loads((ROOT / 'plans/table_transfer_v03/confirmed_start_observation.json').read_text())['robot_state']['configuration']
                initial = start['cartesianPose']
                position_error = math.sqrt(sum((actual[i]-initial[i])**2 for i in (12,13,14)))
                trace = sum(actual[c*4+r]*initial[c*4+r] for c in range(3) for r in range(3))
                rotation_error = math.acos(max(-1.0,min(1.0,(trace-1)/2)))
                joint_error = max(abs(a-b) for a,b in zip(observed['configuration']['jointAngles'],start['jointAngles']))
                profile = observed['configured_end_effector_profile']
                tcp = profile.get('tcp',{})
                ready_workflow = name in ('chat_backward_x_from_franka_ready_v06','chat_flange_local_minus_z_10mm_30_v01','Pick & Place Tomato v2 - Source Approach Fix')
                if ready_workflow and start['jointAngles'] != [0,-math.pi/4,0,-3*math.pi/4,0,math.pi/2,math.pi/4]:
                    raise RuntimeError('The required Franka Ready Home definition changed; execution refused')
                position_limit = .005 if ready_workflow else .01
                rotation_limit = math.radians(1 if ready_workflow else 2)
                joint_limit = math.radians(1) if ready_workflow else .05
                if position_error > position_limit or rotation_error > rotation_limit or joint_error > joint_limit or actual[14] < .2:
                    raise RuntimeError('Arm no longer matches the checked clear starting pose; regenerate the plan before execution')
                if profile.get('deviceId') != 'ee-no-gripper' or any(abs(v)>1e-8 for group in ('translation','rotation') for v in tcp.get(group,{}).values()):
                    raise RuntimeError('The checked bare-flange tool profile changed; execution refused')
                if observed['robot_status'].get('robotErrors'):
                    raise RuntimeError('Robot errors are present; execution refused')
                if observed['robot_status'].get('closeToSingularity') is not False:
                    raise RuntimeError('Onboard singularity warning is active or unavailable; execution refused despite nominal offline checks')
            system = self.reader.get("/api/system")
            if system.get("status") != "Started" or system.get("operatingMode", {}).get("status") != "Execution":
                raise RuntimeError("Robot must already be started and in Execution mode")
            if self.reader.get("/api/fci").get("status") != "Inactive":
                raise RuntimeError("Desk task execution requires FCI to be inactive")
            if self.reader.get("/api/safety/recovery"):
                raise RuntimeError("Robot reports a recovery condition; resolve it locally in Desk")
            if self.reader.get("/api/safety/self-tests").get("status") != "OK":
                raise RuntimeError("Robot self-tests are not OK; resolve this locally in Desk")
            joints = self.reader.get("/api/arm/joints")
            if len(joints) != 7 or any(j.get("brakeStatus") != "Unlocked" for j in joints):
                raise RuntimeError("All seven brakes must already be unlocked")
            execution = self.request("GET", "/desk/api/execution")
            if not isinstance(execution, dict) or execution.get("running") is not False:
                raise RuntimeError("Cannot confirm that Desk task execution is idle; inspect execution status")
            self.request("POST", "/desk/api/execution", {"id": task_id(ident)}, form=True, control=True, start=True)
            return {"task_id": ident, "start_request_accepted": True, "completion_verified": False,
                    "note": "Read franka_real_task_status to inspect execution. An accepted request is not proof of motion or task success"}

    def stop(self):
        with SEND_LOCK:
            STOPPED.set()
        result = self.request("DELETE", "/desk/api/execution", form=True)
        return {"desk_stop_request_accepted": True, "new_task_execution_latched": True, "result": result,
                "scope": "Desk task execution; not a hardware emergency stop or FCI stop"}

    def resume(self):
        with ACTION_LOCK:
            self.session()
            STOPPED.clear()
            return {"new_task_execution_latched": False, "motion_requested": False, "interrupted_tasks_replayed": False}
