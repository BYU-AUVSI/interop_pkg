import unittest
from InteropClient import InteropClient
from ClientObjects import Target

class TestServerConnection(unittest.TestCase):
    def test(self):
        client = InteropClient('localhost', sleep_sec=0.5)

        self.assertIsNotNone(client)
        self.assertTrue(client.is_connected())

        # if we try and connect again it shouldnt affect our connection status:
        client.connect()
        self.assertTrue(client.is_connected())

class TestPostTarget(unittest.TestCase):
    def testManual(self):
        # see how it works with all lower values
        manualTarget = Target('standard', 39.1234, -111.5555, 'ne', 'circle','purple', 'A', 'black', None, False)
        
        client = InteropClient('localhost', sleep_sec=0.0)
        self.assertIsNotNone(client)
        self.assertTrue(client.is_connected())

        raised = False
        result = -1
        try:
            result = client.post_target(manualTarget)
        except:
            raised = True
        self.assertFalse(raised)
        self.assertGreater(result, -1)

    def testAuto(self):
        # see how it works with all caps values
        autoTarget   = Target('STANDARD', 40.1111, -111.6666, 'NE', 'CIRCLE','PURPLE', 'A', 'BLACK', None, True)
        client = InteropClient('localhost', sleep_sec=0.0)
        self.assertIsNotNone(client)
        self.assertTrue(client.is_connected())

        raised = False
        result = -1
        try:
            result = client.post_target(autoTarget)
        except:
            raised = True
        self.assertFalse(raised)
        self.assertGreater(result, -1)

