import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from magnetic_heading import level_heading, tilt_heading
from imu_model import Attitude, Sample, advance, rotate, multiply
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

    def test_tilt_compensation_across_bearings_roll_pitch_and_dip(self):
        def turn(axis, angle):
            a=math.radians(angle)/2
            return (math.cos(a), *(math.sin(a)*v for v in axis))
        for heading in range(0,360,7):
            for roll in (-45,-15,0,15,45):
                for pitch in (-60,-20,0,20,60):
                    q=multiply(turn((0,0,1),-heading),multiply(turn((0,1,0),pitch),turn((1,0,0),roll)))
                    inverse=(q[0], *(-v for v in q[1:]))
                    up=rotate(inverse,(0,0,1))
                    for vertical in (-40,40):
                        field=rotate(inverse,(20,0,vertical))
                        value,reason=tilt_heading(field,up,math.sqrt(2000),up)
                        self.assertIsNotNone(value, reason)
                        self.assertAlmostEqual(((value-heading+180)%360)-180,0,places=9)

    def test_invalid_gravity_vertical_forward_and_inversion(self):
        field=(20,0,-40);reference=math.sqrt(2000)
        for up in (None,(0,0,0),(float('nan'),0,1),(0,0,-1),
                   (math.sin(math.radians(85)),0,math.cos(math.radians(85)))):
            self.assertIsNone(tilt_heading(field,(0,0,1),reference,up)[0])
        self.assertIsNone(tilt_heading((0,0,-40),(0,0,1),40,(0,0,1))[0])
        self.assertIsNone(tilt_heading(field,(0,0,1.5),reference,(0,0,1))[0])

    def test_gyro_gravity_estimate_tracks_a_slow_pitch(self):
        m=Attitude();m.reference()
        for i in range(121):
            m.update(Sample(round(i*1000/60),i,0,(0,0,1),(0,0,0)))
        self.assertTrue(m.ready)
        q=(1,0,0,0);last=2000
        field=(10,math.sqrt(300),-40)  # Magnetic heading 60 degrees.
        for i in range(121,241):
            tick=round(i*1000/60)
            q=advance(q,(0,math.radians(15),0),(tick-last)/1000);last=tick
            inverse=(q[0], *(-v for v in q[1:]))
            up=rotate(inverse,(0,0,1));b=rotate(inverse,field)
            m.update(Sample(tick,i,0,up,(0,15,0)))
            value,reason=tilt_heading(b,m.accel,math.sqrt(2000),m.up_vector())
            self.assertIsNotNone(value,reason)
            self.assertLess(abs(value-60),.5)
        m.update(Sample(last+200,242,0,up,(0,0,0)))
        self.assertIsNone(m.up_vector())

    def test_compass_tilt_mode_requires_reference_and_recovers(self):
        cal=dict(offset_uT=[0,0,0],matrix=[[1,0,0],[0,1,0],[0,0,1]],field_uT=math.sqrt(2000))
        c=Compass(cal,'x,y,z',tilt_comp=True)
        c.update('MAG,100,0,750,0,-1500,25')
        self.assertIsNone(c.heading((0,0,1),100,0)[0])
        self.assertEqual(c.heading((0,0,1),100,0,(0,0,1))[0],0)
        self.assertIsNone(c.heading((0,0,1),100,0,(0,0,-1))[0])
        self.assertEqual(c.heading((0,0,1),100,0,(0,0,1))[0],0)
