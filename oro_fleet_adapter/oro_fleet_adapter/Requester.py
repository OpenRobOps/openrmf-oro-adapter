import requests
from requests import Response
from rclpy.impl.rcutils_logger import RcutilsLogger


class Requester:
    def __init__(self, base_url: str, headers: dict, timeout: float, logger: RcutilsLogger) -> None:
        self.base_url = base_url
        self.headers = headers
        self.timeout = timeout
        self.logger = logger


    def get_request(self, /, * , endpoint: str, json=None) -> Response | None:
        url = f"{self.base_url}{endpoint}"
        url = url.replace("Andino_", "")
        try:
            res = requests.get(url, headers=self.headers, json=json, timeout=self.timeout)
            if res.status_code >= 300:
                self.logger.warn(
                    f"\nStatus code {res.status_code} on GET {url} "
                    f"with body {json}\nmessage: {res.text}")
            return res
        except Exception as e:
            self.logger.error(f"Exception on GET {url}: {e}")
        return None


    def post_request(self, /, * , endpoint: str, json=None) -> Response | None:
        url = f"{self.base_url}{endpoint}"
        url = url.replace("Andino_", "")
        try:
            res = requests.post(url, headers=self.headers, json=json, timeout=self.timeout)
            if res.status_code >= 300:
                self.logger.warn(
                    f"\nStatus code {res.status_code} on POST {url} "
                    f"with body {json}\nmessage: {res.text}")
            return res
        except Exception as e:
            self.logger.error(f"Exception on POST {url}: {e}")
        return None
