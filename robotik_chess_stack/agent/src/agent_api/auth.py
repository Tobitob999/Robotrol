from __future__ import annotations

from fastapi import HTTPException, Request, status


class Auth:
    def __init__(self, psk: str, header: str = "X-PSK"):
        self.psk = psk
        self.header = header

    def verify(self, request: Request) -> None:
        if not self.psk:
            return
        token = request.headers.get(self.header)
        if token != self.psk:
            raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="unauthorized")
