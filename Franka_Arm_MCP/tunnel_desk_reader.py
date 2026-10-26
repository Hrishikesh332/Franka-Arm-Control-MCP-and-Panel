"""Certificate-pinned Desk reader for the local SSH tunnel."""
import hashlib
import hmac
import http.client
import ssl

from franka_config import tunnel_settings
from server import DeskReader


class TunnelDeskReader(DeskReader):
    """Use the local Desk tunnel (127.0.0.1, port from config.json) while preserving the robot identity pin."""

    def __init__(self, config=None):
        super().__init__(config)
        self.tunnel_port = tunnel_settings(self.config)["local_port"]

    def pinned_connection(self):
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
        connection = http.client.HTTPSConnection(
            "127.0.0.1", port=self.tunnel_port, timeout=self.timeout, context=context
        )
        try:
            connection.connect()
            actual = hashlib.sha256(
                connection.sock.getpeercert(binary_form=True)
            ).hexdigest()
            if not hmac.compare_digest(actual, self.pin):
                raise RuntimeError("Desk tunnel presented a different robot certificate")
            return connection
        except OSError as exc:
            connection.close()
            raise RuntimeError(
                f"Could not open the Desk tunnel at 127.0.0.1:{self.tunnel_port} ({type(exc).__name__}). "
                "Start 'Connect Desk over WiFi.command' and keep its window open"
            ) from None
        except Exception:
            connection.close()
            raise
