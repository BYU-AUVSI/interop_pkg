import unittest
from InteropClient import InteropClient

class TestServerConnection(unittest.TestCase):
    def test(self):
        client = InteropClient('localhost', sleep_sec=0.5)

        self.assertIsNone(client)
        self.assertTrue(client.is_connected())

        # if we try and connect again it shouldnt affect our connection status:
        client.connect()
        self.assertTrue(client.is_connected())