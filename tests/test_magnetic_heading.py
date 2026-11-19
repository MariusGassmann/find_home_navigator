import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from magnetic_heading import level_heading
from compass_view import Compass


class HeadingTests(unittest.TestCase):
    def test_all_bearings_and_clockwise_convention(self):
        for angle in range(360):
            a = math.radians(angle)
            value, _ = level_heading((20*math.cos(a),20*math.sin(a),-40),(0,0,1),math.sqrt(2000))
            self.assertAlmostEqual(((value-angle+180)%360)-180,0,places=10)

    def test_cardinal_sensor_to_board_mapping(self):
        cal = dict(offset_uT=[0,0,0],matrix=[[1,0,0],[0,1,0],[0,0,1]],field_uT=math.sqrt(2000))
        # board XYZ = sensor Y,X,-Z. Magnetic north on board +Y gives 90 deg CW heading.
        c = Compass(cal, 'y,x,-z')
        c.update('MAG,100,0,750,0,1500,25')
        self.assertAlmostEqual(c.heading((0,0,1),100,0)[0],90)
        with self.assertRaises(ValueError): Compass(cal, 'y,x,z')

    def test_level_upside_down_acceleration_and_field_guards(self):
        field = (20,0,-40); reference=math.sqrt(2000)
        self.assertIsNotNone(level_heading(field,(math.sin(math.radians(4.9)),0,math.cos(math.radians(4.9))),reference)[0])
        for accel in ((.11,0,.994),(0,0,-1),(0,0,0),(0,0,1.2)):
            self.assertIsNone(level_heading(field,accel,reference)[0])
        for bad in ((0,0,0),(100,0,0),(1,0,-44),(float('nan'),0,-40)):
            self.assertIsNone(level_heading(bad,(0,0,1),reference)[0])
        self.assertIsNone(level_heading(field,(0,0,1),float('inf'))[0])

    def test_freshness_overflow_and_timestamp_wrap(self):
        cal = dict(offset_uT=[0,0,0],matrix=[[1,0,0],[0,1,0],[0,0,1]],field_uT=math.sqrt(2000))
        with patch('compass_view.time.monotonic',return_value=10):
            c = Compass(cal,'x,y,z')
            self.assertIsNone(c.heading((0,0,1),0,0)[0])
            c.update('MAG,4294967280,0,750,0,-1500,25')
            self.assertEqual(c.heading((0,0,1),10,0)[0],0)
            self.assertIsNone(c.heading((0,0,1),1000,0)[0])
            self.assertIsNone(c.heading((0,0,1),None,0)[0])
            self.assertIsNone(c.heading((0,0,1),10,1)[0])
            c.last_data=9
            self.assertIsNone(c.heading((0,0,1),10,0)[0])
            c.update('MAG,100,1,750,0,-1500,3')
            self.assertIsNone(c.heading((0,0,1),100,0)[0])
            c.update('MAG,120,2,750,0,-1500,25')
            self.assertEqual(c.heading((0,0,1),120,0)[0],0)

    def test_no_heading_without_calibration_or_mounting(self):
        c=Compass()
        c.update('MAG,100,0,750,0,-1500,25')
        self.assertIsNone(c.heading((0,0,1),100,0)[0])
