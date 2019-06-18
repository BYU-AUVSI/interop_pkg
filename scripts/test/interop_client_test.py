import unittest
import math
from InteropClient import InteropClient
from ClientObjects import Target, Telemetry

class TestServerConnection(unittest.TestCase):
    def test(self):
        client = InteropClient('localhost', sleep_sec=0.5)

        self.assertIsNotNone(client)
        self.assertTrue(client.is_connected())

        # if we try and connect again it shouldnt affect our connection status:
        client.connect()
        self.assertTrue(client.is_connected())

class TestPostTarget(unittest.TestCase):
    def verifyGoodTargetPost(self, target): #TODO: VALID??
        client = InteropClient('localhost', sleep_sec=0.0)
        self.assertIsNotNone(client)
        self.assertTrue(client.is_connected())

        raised = False
        result = -1
        try:
            result = client.post_target(target)
        except:
            raised = True
        self.assertFalse(raised)
        self.assertGreater(result, -1)

    def testManualStandard(self):
        """
        submit standard manual target
        """
        manualTarget = Target('STANDARD', 39.1234, -111.5555, 'NE', 'CIRCLE','BLUE', 'C', 'BLACK', None, False)
        self.verifyGoodTargetPost(manualTarget)
        

    def testAutoStandard(self):
        """
        submit standard autonomous target
        """
        autoTarget   = Target('STANDARD', 40.1111, -111.6666, 'NE', 'CIRCLE','PURPLE', 'A', 'BLACK', None, True)
        self.verifyGoodTargetPost(autoTarget)

    def testManualEmergent(self):
        """
        submit manual emergent target
        """
        targ = Target('EMERGENT', 40.222, -111.6778, '', '','', '', '', 'A description of an emergent target', False)
        self.verifyGoodTargetPost(targ)
    
    def testAutoEmergent(self):
        """
        submit autonomous emergent target (will this ever actually happen??)
        """
        targ   = Target('EMERGENT', 40.222, -111.6778, '', '','', '', '', 'A description of an emergent target but autonomous', True)
        self.verifyGoodTargetPost(targ)

    def testManualOffAxis(self):
        """
        submit manual off_axis target
        """
        targ   = Target('OFF_AXIS', 42.333, -111.6778, 'W', 'SEMICIRCLE','BROWN', 'F', 'YELLOW', None, False)
        self.verifyGoodTargetPost(targ)
    
    def testAutoOffAxis(self):
        """
        submit autonomous emergent target
        """
        targ = Target('OFF_AXIS', 41.222, -113.6778, 'SW', 'SQUARE','BLUE', 'X', 'WHITE', None, True)
        self.verifyGoodTargetPost(targ)

class TestPostTelem(unittest.TestCase):
    def testPost(self):
        telem = Telemetry(40.111, -111.8888, 1234.1, .5 * math.pi)
        client = InteropClient('localhost', sleep_sec=0.0)
        self.assertIsNotNone(client)
        self.assertTrue(client.is_connected())

        resp = client.post_telemetry(telem)
        self.assertIsNotNone(resp)
        self.assertEqual(resp.status_code, 200) # verify good response