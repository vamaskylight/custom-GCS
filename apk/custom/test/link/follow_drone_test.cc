// Host test for VehicleFollowDrone, the drone side of Follow Me in the app: the
// real code with the real MAVLink headers (the ones the app is built with), and
// stand-ins for QGC's vehicle classes (drone_stubs/).
//
// What comes in: the autopilot's own HEARTBEAT, GPS_RAW_INT and
// GLOBAL_POSITION_INT, as bytes a drone would send. What goes out: the mode
// command and SET_POSITION_TARGET_LOCAL_NED, read back from the message field
// by field and from its bytes. test/follow_sitl_test.py flies the same numbers
// in ArduCopter's simulator.

#include <QtCore/QList>
#include <QtTest/QSignalSpy>
#include <QtTest/QtTest>

#include <cmath>

#include "FollowGuide.h"
#include "MAVLinkLib.h"
#include "MAVLinkProtocol.h"
#include "MultiVehicleManager.h"
#include "Vehicle.h"
#include "VehicleFollowDrone.h"

using skydroid::follow::Drone;
using skydroid::follow::Mode;

Q_DECLARE_METATYPE(skydroid::follow::Drone)

namespace {

constexpr double kPi = 3.14159265358979323846;

// ArduCopter's mode numbers.
constexpr quint32 kStabilize = 0;
constexpr quint32 kAltHold = 2;
constexpr quint32 kAuto = 3;
constexpr quint32 kGuided = 4;
constexpr quint32 kLoiter = 5;
constexpr quint32 kRtl = 6;
constexpr quint32 kLand = 9;
constexpr quint32 kPosHold = 16;
constexpr quint32 kBrake = 17;
constexpr quint32 kSmartRtl = 21;

}  // namespace

class FollowDroneTest : public QObject
{
    Q_OBJECT

private slots:
    void initTestCase() { qRegisterMetaType<skydroid::follow::Drone>(); }

    void init()
    {
        _vehicle = new Vehicle;
        MultiVehicleManager::instance()->setActiveVehicle(_vehicle);
        _drone = new VehicleFollowDrone;
        _told.clear();
        connect(_drone, &FollowDrone::telemetry, this, [this](const Drone &drone) { _told.append(drone); });
    }

    void cleanup()
    {
        delete _drone;
        _drone = nullptr;
        MultiVehicleManager::instance()->setActiveVehicle(nullptr);
        delete _vehicle;
        _vehicle = nullptr;
    }

    // --- Telemetry ----------------------------------------------------------------

    void telemetryComesFromTheAutopilotsOwnMessages()
    {
        _heartbeat(kLoiter, true);
        _gps(GPS_FIX_TYPE_3D_FIX);
        QVERIFY(_told.isEmpty());  // the position is the telemetry: nothing is told before it
        // 19.8816785 N, 73.8199197 E, 31.5 m above the take-off point, 1.2 m/s north and 3.4 m/s west, nose to 271.5 degrees.
        _position(198816785, 738199197, 31500, 120, -340, 27150);
        QCOMPARE(_told.size(), 1);
        const Drone d = _told.constLast();
        QCOMPARE(d.latDeg, 19.8816785);
        QCOMPARE(d.lonDeg, 73.8199197);
        QCOMPARE(d.heightM, 31.5);
        QCOMPARE(d.northMps, 1.2);
        QCOMPARE(d.eastMps, -3.4);
        QCOMPARE(d.headingDeg, 271.5);
        QVERIFY(d.armed && d.flying && d.gps3d);
        QCOMPARE(d.mode, Mode::Holding);
    }

    void theModeIsArduCoptersOwnNumber_data()
    {
        QTest::addColumn<quint32>("number");
        QTest::addColumn<int>("mode");
        QTest::newRow("Guided") << kGuided << static_cast<int>(Mode::Guided);
        QTest::newRow("Loiter") << kLoiter << static_cast<int>(Mode::Holding);
        QTest::newRow("Position Hold") << kPosHold << static_cast<int>(Mode::Holding);
        QTest::newRow("Brake") << kBrake << static_cast<int>(Mode::Holding);
        QTest::newRow("Stabilize") << kStabilize << static_cast<int>(Mode::Other);
        QTest::newRow("Altitude Hold") << kAltHold << static_cast<int>(Mode::Other);
        QTest::newRow("Auto") << kAuto << static_cast<int>(Mode::Other);
        QTest::newRow("RTL") << kRtl << static_cast<int>(Mode::Other);
        QTest::newRow("Land") << kLand << static_cast<int>(Mode::Other);
        QTest::newRow("Smart RTL") << kSmartRtl << static_cast<int>(Mode::Other);
    }

    void theModeIsArduCoptersOwnNumber()
    {
        QFETCH(quint32, number);
        QFETCH(int, mode);
        // QGC's name for the mode is in the app's language; the number is not.
        _vehicle->mode = QStringLiteral("whatever the language calls it");
        _heartbeat(number, true);
        _position(198816785, 738199197, 30000, 0, 0, 0);
        QCOMPARE(static_cast<int>(_told.constLast().mode), mode);
        QCOMPARE(_drone->modeName(), QString("whatever the language calls it"));
    }

    void whatTheDroneDoesNotSayIsNotMadeUp()
    {
        // No heartbeat and no GPS message yet: not armed, no 3D fix, no mode Follow could start from.
        _position(198816785, 738199197, 30000, 0, 0, UINT16_MAX);
        QCOMPARE(_told.size(), 1);
        QVERIFY(!_told.constLast().armed && !_told.constLast().gps3d);
        QCOMPARE(_told.constLast().mode, Mode::Other);
        QVERIFY(std::isnan(_told.constLast().headingDeg));  // 65535: the drone does not know its heading
        // On the ground, a 2D fix, disarmed.
        _vehicle->inTheAir = false;
        _heartbeat(kLoiter, false);
        _gps(GPS_FIX_TYPE_2D_FIX);
        _position(198816785, 738199197, 0, 0, 0, 9000);
        QVERIFY(!_told.constLast().armed && !_told.constLast().flying && !_told.constLast().gps3d);
        QCOMPARE(_told.constLast().headingDeg, 90.0);
        // A better fix than 3D is a 3D fix.
        _gps(GPS_FIX_TYPE_RTK_FIXED);
        _position(198816785, 738199197, 0, 0, 0, 9000);
        QVERIFY(_told.constLast().gps3d);
    }

    void aPositionOfNoughtNoughtIsNoPosition()
    {
        // ArduPilot sends GLOBAL_POSITION_INT without a GPS position as well, for the height alone.
        _heartbeat(kLoiter, true);
        _position(0, 0, 30000, 0, 0, 0);
        QVERIFY(_told.isEmpty());
    }

    void messagesOfOtherSendersAreNotTheDrones()
    {
        _heartbeat(kLoiter, true);
        // A gimbal on the same drone says "mode 4" in its own heartbeat. That is not the autopilot's mode.
        _heartbeat(kGuided, false, 1, MAV_COMP_ID_GIMBAL);
        // Another drone's position.
        _position(100000000, 200000000, 5000, 0, 0, 0, 2, 1);
        QVERIFY(_told.isEmpty());
        // A companion computer's position message for this drone's system.
        _position(100000000, 200000000, 5000, 0, 0, 0, 1, MAV_COMP_ID_ONBOARD_COMPUTER);
        QVERIFY(_told.isEmpty());
        _position(198816785, 738199197, 30000, 0, 0, 0);
        QCOMPARE(_told.size(), 1);
        QCOMPARE(_told.constLast().mode, Mode::Holding);
        QVERIFY(_told.constLast().armed);
    }

    void anotherDroneStartsFromNothing()
    {
        _heartbeat(kGuided, true);
        _gps(GPS_FIX_TYPE_3D_FIX);
        QSignalSpy others(_drone, &FollowDrone::otherDrone);
        _position(198816785, 738199197, 30000, 0, 0, 0);
        QCOMPARE(others.count(), 0);  // its messages do not make it another drone
        _told.clear();
        Vehicle other;
        other.systemId = 7;
        MultiVehicleManager::instance()->setActiveVehicle(&other);
        QCOMPARE(others.count(), 1);  // Follow is told, and ends
        // The first drone goes on talking: it is not the one any more.
        _position(198816785, 738199197, 30000, 0, 0, 0);
        QVERIFY(_told.isEmpty());
        // The new one has said nothing but its position yet: nothing of the first one counts for it.
        mavlink_message_t message;
        mavlink_msg_global_position_int_pack(7, 1, &message, 1000, 198800000, 738100000, 600000, 20000, 0, 0, 0, 0);
        other.receive(message);
        QCOMPARE(_told.size(), 1);
        QVERIFY(!_told.constLast().armed && !_told.constLast().gps3d);
        QCOMPARE(_told.constLast().mode, Mode::Other);
        // QGC says "the active drone" again with the same drone: that is no other drone, and what
        // it has said of itself still holds.
        mavlink_msg_heartbeat_pack(7, 1, &message, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA,
                                   MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED, kGuided, MAV_STATE_ACTIVE);
        other.receive(message);
        MultiVehicleManager::instance()->setActiveVehicle(&other);
        QCOMPARE(others.count(), 1);
        mavlink_msg_global_position_int_pack(7, 1, &message, 2000, 198800000, 738100000, 600000, 20000, 0, 0, 0, 0);
        other.receive(message);
        QCOMPARE(_told.size(), 2);  // once, not twice: it is listened to once
        QVERIFY(_told.constLast().armed);
        QCOMPARE(_told.constLast().mode, Mode::Guided);
        // A drone with the same system id as the first (on another link): the first is not listened to any more.
        Vehicle twin;
        MultiVehicleManager::instance()->setActiveVehicle(&twin);
        QCOMPARE(others.count(), 2);
        _told.clear();
        _position(198816785, 738199197, 30000, 0, 0, 0);
        QVERIFY(_told.isEmpty());
        // No drone at all is another drone too.
        MultiVehicleManager::instance()->setActiveVehicle(nullptr);
        QCOMPARE(others.count(), 3);
        MultiVehicleManager::instance()->setActiveVehicle(_vehicle);
    }

    // --- Which drones Follow flies -------------------------------------------------

    void onlyAnArduPilotMulticopterIsFlown()
    {
        QVERIFY(_drone->whyNot().isEmpty());
        _vehicle->copter = false;  // a plane, a rover
        QVERIFY2(_drone->whyNot().contains("ArduPilot multicopter only"), qPrintable(_drone->whyNot()));
        _heartbeat(kGuided, true);  // on a plane, 4 is not Guided
        _position(198816785, 738199197, 30000, 0, 0, 0);
        QCOMPARE(_told.constLast().mode, Mode::Other);
        _drone->askGuided();
        _drone->sendSpeed(1.0, 1.0, 0.0);
        QVERIFY(_vehicle->commands.isEmpty() && _vehicle->sent.isEmpty());
        _vehicle->copter = true;
        _vehicle->ardupilot = false;  // PX4: its modes and its offboard rules are others
        QVERIFY(_drone->whyNot().contains("ArduPilot multicopter only"));
        _drone->askGuided();
        _drone->sendSpeed(1.0, 1.0, 0.0);
        QVERIFY(_vehicle->commands.isEmpty() && _vehicle->sent.isEmpty());
    }

    void withNoDroneNothingIsSent()
    {
        MultiVehicleManager::instance()->setActiveVehicle(nullptr);
        QVERIFY2(_drone->whyNot().contains("no drone is connected"), qPrintable(_drone->whyNot()));
        QVERIFY(_drone->modeName().isEmpty());
        QCOMPARE(_drone->fliesOnS(), -1.0);
        _drone->askGuided();
        _drone->sendSpeed(1.0, 1.0, 0.0);
        QVERIFY(_vehicle->commands.isEmpty() && _vehicle->sent.isEmpty());
    }

    // --- What is sent --------------------------------------------------------------

    void guidedIsAskedForWithArduCoptersNumber()
    {
        _drone->askGuided();
        QCOMPARE(_vehicle->commands.size(), 1);
        const Vehicle::Command c = _vehicle->commands.constFirst();
        QCOMPARE(c.component, 1);
        QCOMPARE(static_cast<int>(c.command), static_cast<int>(MAV_CMD_DO_SET_MODE));
        QCOMPARE(c.param1, static_cast<float>(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED));
        QCOMPARE(c.param2, 4.0f);
        QVERIFY(!c.showError);  // Follow says itself when the drone did not change
        QVERIFY(_vehicle->sent.isEmpty());
    }

    void aSpeedGoesOutWithTheHeadingToKeepAndNothingElse()
    {
        _vehicle->systemId = 3;
        _vehicle->componentId = 1;
        _vehicle->links.link->channel = 2;
        _drone->sendSpeed(1.5, -2.25, 135.0);
        QCOMPARE(_vehicle->sent.size(), 1);
        QCOMPARE(_vehicle->sentOn.constFirst(), _vehicle->links.link.get());
        const mavlink_message_t message = _vehicle->sent.constFirst();
        QCOMPARE(static_cast<int>(message.msgid), MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED);
        // From the ground station, as QGC's own commands.
        QCOMPARE(static_cast<int>(message.sysid), 255);
        QCOMPARE(static_cast<int>(message.compid), static_cast<int>(MAV_COMP_ID_MISSIONPLANNER));
        mavlink_set_position_target_local_ned_t t;
        mavlink_msg_set_position_target_local_ned_decode(&message, &t);
        QCOMPARE(static_cast<int>(t.target_system), 3);
        QCOMPARE(static_cast<int>(t.target_component), 1);
        // North and east over the ground.
        QCOMPARE(static_cast<int>(t.coordinate_frame), static_cast<int>(MAV_FRAME_LOCAL_NED));
        QCOMPARE(t.vx, 1.5f);
        QCOMPARE(t.vy, -2.25f);
        QCOMPARE(t.vz, 0.0f);  // the height is kept
        // Used: the speed and the yaw angle. Not used: position, acceleration, yaw rate.
        const int mask = POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE | POSITION_TARGET_TYPEMASK_Z_IGNORE |
                         POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                         POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
        QCOMPARE(static_cast<int>(t.type_mask), mask);
        QCOMPARE(static_cast<int>(t.type_mask), 0x09C7);
        QVERIFY2(std::abs(t.yaw - 135.0 * kPi / 180.0) < 1e-6, qPrintable(QString::number(t.yaw)));
        QCOMPARE(t.yaw_rate, 0.0f);
        QCOMPARE(t.x, 0.0f);
        QCOMPARE(t.y, 0.0f);
        QCOMPARE(t.z, 0.0f);
        QCOMPARE(t.afx, 0.0f);
        QCOMPARE(t.afy, 0.0f);
        QCOMPARE(t.afz, 0.0f);

        // The message as the bytes that go to the drone, read back as the drone reads them.
        uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
        const uint16_t length = mavlink_msg_to_send_buffer(bytes, &message);
        mavlink_message_t back;
        mavlink_status_t status;
        bool whole = false;
        for (uint16_t i = 0; i < length; ++i) {
            whole = mavlink_parse_char(MAVLINK_COMM_3, bytes[i], &back, &status) != 0;
        }
        QVERIFY(whole);  // length and checksum are right
        mavlink_set_position_target_local_ned_t again;
        mavlink_msg_set_position_target_local_ned_decode(&back, &again);
        QCOMPARE(again.vx, 1.5f);
        QCOMPARE(again.vy, -2.25f);
        QCOMPARE(static_cast<int>(again.type_mask), 0x09C7);
    }

    void theHeadingGoesOutAsAnAngleFromNorthBetweenMinusAndPlusPi_data()
    {
        QTest::addColumn<double>("heading");
        QTest::addColumn<double>("yawDeg");
        QTest::newRow("north") << 0.0 << 0.0;
        QTest::newRow("east") << 90.0 << 90.0;
        QTest::newRow("south") << 180.0 << 180.0;
        QTest::newRow("west") << 270.0 << -90.0;
        QTest::newRow("just short of north") << 359.5 << -0.5;
        QTest::newRow("counted the other way") << -45.0 << -45.0;
        QTest::newRow("a turn too many") << 405.0 << 45.0;
    }

    void theHeadingGoesOutAsAnAngleFromNorthBetweenMinusAndPlusPi()
    {
        QFETCH(double, heading);
        QFETCH(double, yawDeg);
        _drone->sendSpeed(0.0, 0.0, heading);
        const mavlink_set_position_target_local_ned_t t = _lastTarget();
        QCOMPARE(static_cast<int>(t.type_mask), 0x09C7);
        // South is +180 or -180: the same heading.
        const double off = std::remainder(t.yaw * 180.0 / kPi - yawDeg, 360.0);
        QVERIFY2(std::abs(off) < 1e-4, qPrintable(QString::number(t.yaw * 180.0 / kPi)));
        QVERIFY(std::abs(t.yaw) <= kPi + 1e-6);
    }

    void withNoHeadingTheSpeedGoesOutWithAYawRateOfNought()
    {
        _drone->sendSpeed(2.0, 3.0, std::nan(""));
        const mavlink_set_position_target_local_ned_t t = _lastTarget();
        const int mask = POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE | POSITION_TARGET_TYPEMASK_Z_IGNORE |
                         POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                         POSITION_TARGET_TYPEMASK_YAW_IGNORE;
        QCOMPARE(static_cast<int>(t.type_mask), mask);
        QCOMPARE(static_cast<int>(t.type_mask), 0x05C7);
        QCOMPARE(t.yaw_rate, 0.0f);
        QCOMPARE(t.yaw, 0.0f);
        QCOMPARE(t.vx, 2.0f);
        QCOMPARE(t.vy, 3.0f);
    }

    void aSpeedThatIsNoNumberGoesOutAsStand()
    {
        _drone->sendSpeed(std::nan(""), 4.0, 10.0);
        mavlink_set_position_target_local_ned_t t = _lastTarget();
        QCOMPARE(t.vx, 0.0f);
        QCOMPARE(t.vy, 0.0f);
        _drone->sendSpeed(4.0, INFINITY, 10.0);
        t = _lastTarget();
        QCOMPARE(t.vx, 0.0f);
        QCOMPARE(t.vy, 0.0f);
        QCOMPARE(t.vz, 0.0f);
    }

    void withTheLinkGoneNothingIsSent()
    {
        _vehicle->links.link.reset();
        _drone->sendSpeed(1.0, 1.0, 0.0);
        QVERIFY(_vehicle->sent.isEmpty());
    }

    void theNumbersAreTheOnesFlownInTheSimulator()
    {
        // test/follow_sitl_test.py takes them from FollowGuide.h through the guide's DLL.
        QCOMPARE(static_cast<int>(skydroid::follow::kSpeedTypeMask), 0x09C7);
        QCOMPARE(static_cast<int>(skydroid::follow::kSpeedTypeMaskNoHeading), 0x05C7);
        QCOMPARE(static_cast<int>(skydroid::follow::kSpeedFrame), static_cast<int>(MAV_FRAME_LOCAL_NED));
        QCOMPARE(static_cast<quint32>(skydroid::follow::kCopterGuided), kGuided);
        QCOMPARE(static_cast<quint32>(skydroid::follow::kCopterLoiter), kLoiter);
        QCOMPARE(static_cast<quint32>(skydroid::follow::kCopterPosHold), kPosHold);
        QCOMPARE(static_cast<quint32>(skydroid::follow::kCopterBrake), kBrake);
    }

    // --- How long the drone flies on -----------------------------------------------

    void howLongTheDroneFliesOnIsItsOwnParameter()
    {
        QCOMPARE(_drone->fliesOnS(), -1.0);  // the parameters are not read yet
        _vehicle->parameters.set("GUID_TIMEOUT", 3.0);
        QCOMPARE(_drone->fliesOnS(), 3.0);
        _vehicle->parameters.set("GUID_TIMEOUT", 1.5);
        QCOMPARE(_drone->fliesOnS(), 1.5);
        _vehicle->parameters.set("GUID_TIMEOUT", "nonsense");
        QCOMPARE(_drone->fliesOnS(), -1.0);
    }

private:
    void _heartbeat(quint32 customMode, bool armed, int system = 1, int component = 1)
    {
        mavlink_message_t message;
        const uint8_t base = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0);
        mavlink_msg_heartbeat_pack(static_cast<uint8_t>(system), static_cast<uint8_t>(component), &message,
                                   MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA, base, customMode, MAV_STATE_ACTIVE);
        _vehicle->receive(message);
    }

    void _gps(int fixType)
    {
        mavlink_message_t message;
        mavlink_msg_gps_raw_int_pack(1, 1, &message, 0, static_cast<uint8_t>(fixType), 198816785, 738199197, 630000, 80, 120, 0,
                                     0, 14, 0, 0, 0, 0, 0, 0);
        _vehicle->receive(message);
    }

    /// lat, lon in 1e-7 degrees, the height above the take-off point in mm, speeds in cm/s, the heading in 0.01 degrees.
    void _position(qint32 lat, qint32 lon, qint32 heightMm, int northCms, int eastCms, int headingCdeg, int system = 1,
                   int component = 1)
    {
        mavlink_message_t message;
        mavlink_msg_global_position_int_pack(static_cast<uint8_t>(system), static_cast<uint8_t>(component), &message, 1000,
                                             lat, lon, 630000, heightMm, static_cast<int16_t>(northCms),
                                             static_cast<int16_t>(eastCms), 0, static_cast<uint16_t>(headingCdeg));
        _vehicle->receive(message);
    }

    mavlink_set_position_target_local_ned_t _lastTarget() const
    {
        mavlink_set_position_target_local_ned_t t{};
        if (!_vehicle->sent.isEmpty()) {
            mavlink_msg_set_position_target_local_ned_decode(&_vehicle->sent.constLast(), &t);
        }
        return t;
    }

    Vehicle *_vehicle = nullptr;
    VehicleFollowDrone *_drone = nullptr;
    QList<Drone> _told;
};

QTEST_GUILESS_MAIN(FollowDroneTest)
#include "follow_drone_test.moc"
