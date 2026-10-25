"""Minimal client for the Franka Desk web API: take control, unlock the brakes, activate FCI.

Desk has no public API documentation; these are the endpoints its own web page uses (the same
ones tools like panda-py use). Only one session can hold control of the robot. If someone else
holds it (e.g. a Desk browser tab), control can be taken over by confirming with the circle button
on the robot's Pilot.
"""

import base64
import hashlib
import time

import requests
import urllib3

urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)  # Desk uses a self-signed cert


class DeskError(RuntimeError):
    pass


class Desk:

    def __init__(self, host, user, password, timeout=10.0):
        self.base = f'https://{host}'
        self.user = user
        self.password = password
        self.timeout = timeout
        self.session = requests.Session()
        self.session.verify = False
        self.token = None      # control token
        self.token_id = None

    @staticmethod
    def encode_password(user, password):
        digest = hashlib.sha256(f'{password}#{user}@franka'.encode()).digest()
        return base64.encodebytes(','.join(str(b) for b in digest).encode()).decode()

    def _request(self, method, path, **kw):
        headers = kw.pop('headers', {})
        if self.token:
            headers['X-Control-Token'] = self.token
        r = self.session.request(method, self.base + path, headers=headers, timeout=self.timeout, **kw)
        if r.status_code >= 400:
            raise DeskError(f'{method} {path}: HTTP {r.status_code} {r.text.strip()[:200]}')
        return r

    def login(self):
        r = self._request('POST', '/admin/api/login',
                          json={'login': self.user, 'password': self.encode_password(self.user, self.password)})
        self.session.cookies.set('authorization', r.text.strip().strip('"'))

    def active_token_id(self):
        r = self._request('GET', '/admin/api/control-token')
        active = (r.json() or {}).get('activeToken') or {}
        return active.get('id')

    def take_control(self, on_waiting=None, wait=20.0):
        """Request control; if someone else holds it, ask for confirmation on the Pilot."""
        # A request always succeeds, but while someone else (e.g. a Desk browser tab) holds control
        # it is only queued: we have control only once our token is the active one.
        r = self._request('POST', '/admin/api/control-token/request', json={'requestedBy': self.user})
        self.token, self.token_id = r.json()['token'], r.json()['id']
        if self.active_token_id() == self.token_id:
            return
        r = self._request('POST', '/admin/api/control-token/request?force', json={'requestedBy': self.user})
        self.token, self.token_id = r.json()['token'], r.json()['id']
        if on_waiting:
            on_waiting('press the circle button on the robot\'s Pilot to confirm taking control')
        end = time.time() + wait
        while time.time() < end:
            if self.active_token_id() == self.token_id:
                return
            time.sleep(0.5)
        raise DeskError('control was not confirmed on the Pilot in time')

    def release_control(self):
        if self.token:
            try:
                self._request('DELETE', '/admin/api/control-token', json={'token': self.token})
            finally:
                self.token = self.token_id = None

    def open_brakes(self):
        self._request('POST', '/desk/api/robot/open-brakes')

    def close_brakes(self):
        self._request('POST', '/desk/api/robot/close-brakes')

    def activate_fci(self):
        self._request('POST', '/admin/api/control-token/fci', json={'token': self.token})

    def deactivate_fci(self):
        self._request('DELETE', '/admin/api/control-token/fci', json={'token': self.token})

    def system_version(self):
        return self._request('GET', '/admin/api/system-version').json().split()[0]
