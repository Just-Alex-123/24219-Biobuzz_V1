package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * ONE-FILE TELEOP: press START and both turrets turn to face TARGET_X / TARGET_Y and stay on it.
 * No subsystem, no gamepad needed.
 *
 * ASSUMPTIONS (change if wrong):
 *  - Each turret is driven by a continuous-rotation servo (CRServo): "left_T" and "right_T".
 *  - Each turret has an analog absolute encoder: "left_T_enc" and "right_T_enc" (rename to your config).
 *  - Angles are DEGREES, counter-clockwise positive, 0 = robot forward (Pedro: x forward, y left).
 *  - Field positions are inches, heading is radians (Pedro's convention).
 *  - Pedro Pathing 3 (Constants.create, follower.setPose, follower.pose(), follower.velocity()).
 *    Only the lines marked "PEDRO" below depend on it.
 *
 * THE MATH, in short:
 *  1. AIMING: each turret has its own mount offset, so we find each turret's own spot on the field
 *     and aim from THERE. That is what makes both turrets converge on one point.
 *  2. WRAPPING: each turret can travel a bit MORE than 360 deg. The ends of that range (the "seam")
 *     are on the OUTER side (left turret: left side, right turret: right side). When the target
 *     crosses the seam, the turret unwinds. In the overlap zone it keeps the lap it is on.
 *  3. CONTROL: PID + static friction + velocity feedforward (keeps tracking while the robot moves).
 *
 * FIRST RUN ORDER (do this BEFORE trusting it):
 *  a) Set USE_ODOMETRY = false (bench mode: the robot pose is fixed to START_X/Y/HEADING, no Pedro).
 *  b) Telemetry shows rawEncDeg. Hand-rotate each turret, fix ENC_ZERO and ENC_SIGN.
 *  c) Fix each POWER_SIGN so it turns TOWARD the target, not away.
 *  d) Tune the gains (KS -> KP -> KD -> KV -> KI). Keep MAX_POWER low at first.
 *  e) Then set USE_ODOMETRY = true.
 */
@TeleOp
public class turret_v1 extends LinearOpMode {

    // =====================================================================
    // TARGET + ROBOT START POSE
    // TARGET_X/Y: the field point (inches) both turrets look at.
    // START_*: where the robot is placed when you press INIT (inches, radians), used by Pedro as the
    //   starting pose, or as the fixed pose in bench mode. Tune: place the robot exactly there.
    // USE_ODOMETRY: true = read the pose and velocity from Pedro. false = bench mode, robot pose stays at START.
    // =====================================================================
    public static double TARGET_X = 132.0;
    public static double TARGET_Y = 12.0;
    public static double START_X = 72.0;
    public static double START_Y = 72.0;
    public static double START_HEADING_RAD = 0.0;
    public static boolean USE_ODOMETRY = false;

    // =====================================================================
    // TUNE 1: GEOMETRY (measure on the robot / CAD)
    // Each turret's CENTER OF ROTATION relative to the robot center (the point odometry reports),
    // in INCHES. +X = forward, +Y = left. If wrong, the two turrets aim at slightly different
    // points, and the error is worse the closer you are to the target.
    // =====================================================================
    public static double LEFT_MOUNT_X  = 0.0;
    public static double LEFT_MOUNT_Y  = 4.5;
    public static double RIGHT_MOUNT_X = 0.0;
    public static double RIGHT_MOUNT_Y = -4.5;

    // =====================================================================
    // TUNE 2: TRAVEL LIMITS / WRAP
    // Turret travel = 360 + OVERLAP_DEG. Hard stops/wires must allow at least that.
    // Bigger overlap = fewer unwinds when the target hovers near the outer side. Start at 20;
    // the physical stop must be OUTSIDE the software limit.
    // SOFT_LIMIT_MARGIN_DEG: within this many degrees of a limit the turret slows to zero so it never
    //   slams the stop. Increase if it hits the stop, decrease if it feels sluggish near the edges.
    // *_SEAM_DEG: robot-frame direction of each turret's seam. Left = +90, right = -90 (outer sides).
    //   Swap them to wrap on the INSIDE instead.
    // =====================================================================
    public static double OVERLAP_DEG = 20.0;
    public static double SOFT_LIMIT_MARGIN_DEG = 8.0;
    public static double LEFT_SEAM_DEG  = 90.0;
    public static double RIGHT_SEAM_DEG = -90.0;

    // =====================================================================
    // TUNE 3: ENCODER
    // ENCODER_REVS_PER_TURRET_REV: encoder shaft turns per 1 turret turn.
    //   Encoder on the turret = 1.0. Encoder on the servo shaft = (ring teeth / pinion teeth),
    //   e.g. 100T/20T = 5.0. Check: rotate the turret exactly one turn by hand and watch rawEncDeg.
    // ENCODER_MAX_VOLTS: 3.3 for most analog encoders (Axon = 3.3). Verify on the Driver Hub.
    // *_ENC_ZERO_DEG: the rawEncDeg reading when that turret points straight FORWARD.
    //   Tune: point the turret forward by eye, read telemetry, type it in.
    // *_ENC_SIGN: +1 if the turret angle goes up (turns CCW seen from above) when the encoder angle
    //   goes up, otherwise -1. Check by hand-rotating the turret CCW and watching the angle in telemetry.
    // INIT_HINT_DEG: roughly where the turrets are at power-on (0 = forward). Only matters when the
    //   encoder turns more than once per turret turn (ratio > 1). Needs to be within (180 / ratio) deg.
    // =====================================================================
    public static double ENCODER_REVS_PER_TURRET_REV = 1.0;
    public static double ENCODER_MAX_VOLTS = 3.3;
    public static double LEFT_ENC_ZERO_DEG  = 0.0;
    public static double RIGHT_ENC_ZERO_DEG = 0.0;
    public static double LEFT_ENC_SIGN  = 1.0;
    public static double RIGHT_ENC_SIGN = 1.0;
    public static double INIT_HINT_DEG = 0.0;

    // =====================================================================
    // TUNE 4: SERVO DIRECTION (+1 or -1)
    // If a turret runs AWAY from its target (spins to the stop), flip that turret's sign.
    // Do this BEFORE touching any gains.
    // =====================================================================
    public static double LEFT_POWER_SIGN  = 1.0;
    public static double RIGHT_POWER_SIGN = 1.0;

    // =====================================================================
    // TUNE 5: CONTROL GAINS (tune in this order: KS -> KP -> KD -> KV -> KI)
    //
    // MAX_POWER: cap on servo power (0..1). Start at 0.4 while testing.
    //
    // KS (static friction): power needed just to get the turret creeping.
    //   Tune: set KP=KD=KV=0, give a small constant error, raise KS until it barely starts moving,
    //   then back off about 20%. Too high = jitter around the target.
    //
    // KP (power per degree of error):
    //   Tune: start at 0.01. Raise until it snaps to the target quickly. If it oscillates, cut ~30%.
    //
    // KD (power per deg/s of velocity error): damping.
    //   Tune: with KP set, raise KD until overshoot settles. Start near 0.0005. Too high = noisy/buzzing.
    //
    // KV (power per deg/s of feedforward velocity): makes it track while the robot moves.
    //   Tune: rotate the robot in place while the turret aims at a point, raise KV until the aim error
    //   stays near zero. First guess: KV = 1 / (turret max speed in deg/s at full power);
    //   measure that speed with power = 1 and a stopwatch on the encoder data.
    //
    // KI (power per degree*second of accumulated error): fixes a small steady offset.
    //   Tune: leave 0 unless it parks a few degrees off target; then try 0.002.
    //   Only works within I_ZONE_DEG, capped by I_MAX_POWER.
    //
    // DEADBAND_DEG: below this error the turret stops adding KS (stops buzzing).
    //   Tune: raise until it stops jittering on target, but keep it below what your shot needs.
    //
    // ON_TARGET_TOL_DEG: how close counts as "on target" in telemetry. Tune to shot tolerance
    //   (roughly atan(goal_radius / distance) in degrees).
    //
    // VEL_FILTER_ALPHA: 0..1 smoothing of the measured velocity. Lower = smoother but laggier.
    //   Start at 0.3; if D is noisy, lower it.
    // =====================================================================
    public static double MAX_POWER = 0.4;
    public static double KS = 0.05;
    public static double KP = 0.01;
    public static double KD = 0.0005;
    public static double KV = 0.0008;
    public static double KI = 0.0;
    public static double I_ZONE_DEG = 5.0;
    public static double I_MAX_POWER = 0.1;
    public static double DEADBAND_DEG = 0.5;
    public static double ON_TARGET_TOL_DEG = 1.5;
    public static double VEL_FILTER_ALPHA = 0.3;

    // =====================================================================

    @Override
    public void runOpMode() {
        Turret left = new Turret(hardwareMap.get(CRServo.class, "left_T"),
                hardwareMap.get(AnalogInput.class, "left_T_enc"), true);
        Turret right = new Turret(hardwareMap.get(CRServo.class, "right_T"),
                hardwareMap.get(AnalogInput.class, "right_T_enc"), false);

        // PEDRO: only build the follower when we use odometry
        Follower follower = null;
        if (USE_ODOMETRY) {
            follower = Constants.create(hardwareMap);                          // PEDRO 3 (was createFollower)
            follower.setPose(new Pose(START_X, START_Y, START_HEADING_RAD));   // PEDRO 3 (was setStartingPose); heading in radians
        }

        ElapsedTime timer = new ElapsedTime();

        // While waiting for START, show the raw encoders so you can set ENC_ZERO / ENC_SIGN.
        while (!isStarted() && !isStopRequested()) {
            left.readEncoder(0.02);
            right.readEncoder(0.02);
            telemetry.addLine("Ready. Press START to aim both turrets.");
            telemetry.addData("left rawEncDeg", "%.1f", left.rawEncDeg);
            telemetry.addData("right rawEncDeg", "%.1f", right.rawEncDeg);
            telemetry.addData("left angle", "%.1f", left.angleDeg);
            telemetry.addData("right angle", "%.1f", right.angleDeg);
            telemetry.update();
        }
        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            double dt = Math.max(1e-3, Math.min(timer.seconds(), 0.1));
            timer.reset();

            // ---- robot pose + velocity ----
            double x = START_X, y = START_Y, h = START_HEADING_RAD;
            double vx = 0, vy = 0, omega = 0;
            if (USE_ODOMETRY && follower != null) {
                follower.update();                       // PEDRO
                Pose p = follower.pose();                // PEDRO 3 (was getPose): inches, radians
                x = p.x();
                y = p.y();
                h = p.heading();
                // PEDRO 3: velocity() is the world-frame velocity (immutable object with vx, vy, omega)
                vx = follower.velocity().vx;             // in/s, field frame
                vy = follower.velocity().vy;
                omega = follower.velocity().omega;       // rad/s, CCW+
            }

            left.update(dt, x, y, h, vx, vy, omega, TARGET_X, TARGET_Y);
            right.update(dt, x, y, h, vx, vy, omega, TARGET_X, TARGET_Y);

            telemetry.addData("mode", USE_ODOMETRY ? "odometry" : "BENCH (fixed pose)");
            telemetry.addData("robot", "x %.1f  y %.1f  h %.1f deg", x, y, Math.toDegrees(h));
            telemetry.addData("target", "%.1f, %.1f", TARGET_X, TARGET_Y);
            telemetry.addLine("--- LEFT ---");
            left.addTelemetry(telemetry);
            telemetry.addLine("--- RIGHT ---");
            right.addTelemetry(telemetry);
            telemetry.update();
        }

        left.stop();
        right.stop();
    }

    // =====================================================================
    // One turret. All the math lives here.
    // =====================================================================
    private static class Turret {
        final CRServo servo;
        final AnalogInput enc;
        final boolean isLeft;

        double rawEncDeg;       // 0..360 raw absolute reading
        double lastRawEncDeg;
        double angleDeg;        // continuous turret angle, robot frame, CCW+, 0 = forward
        double lastAngleDeg;
        double measVelDegPerSec;
        double desiredDeg;      // aim angle wrapped to (-180, 180]
        double cmdDeg;          // chosen lap of that angle, inside the travel limits
        double errDeg;
        double ffVelDeg;
        double integral;
        double lastPower;
        boolean initialized = false;

        Turret(CRServo servo, AnalogInput enc, boolean isLeft) {
            this.servo = servo;
            this.enc = enc;
            this.isLeft = isLeft;
        }

        double mountX()  { return isLeft ? LEFT_MOUNT_X : RIGHT_MOUNT_X; }
        double mountY()  { return isLeft ? LEFT_MOUNT_Y : RIGHT_MOUNT_Y; }
        double seamDeg() { return isLeft ? LEFT_SEAM_DEG : RIGHT_SEAM_DEG; }
        double encZero() { return isLeft ? LEFT_ENC_ZERO_DEG : RIGHT_ENC_ZERO_DEG; }
        double encSign() { return isLeft ? LEFT_ENC_SIGN : RIGHT_ENC_SIGN; }
        double pwrSign() { return isLeft ? LEFT_POWER_SIGN : RIGHT_POWER_SIGN; }

        /**
         * Travel limits in the robot frame: (360 + OVERLAP) wide, and one END of the range is the seam.
         * Left  (seam +90): [-270, 90 + overlap]  -> hits its end on the LEFT (outer) side.
         * Right (seam -90): [-90 - overlap, 270]  -> hits its end on the RIGHT (outer) side.
         */
        double minDeg() {
            double seam = seamDeg();
            return seam > 0 ? seam - 360.0 : seam - OVERLAP_DEG;
        }

        double maxDeg() {
            double seam = seamDeg();
            return seam > 0 ? seam + OVERLAP_DEG : seam + 360.0;
        }

        void stop() {
            servo.setPower(0);
            integral = 0;
            lastPower = 0;
        }

        // ---- ENCODER: absolute reading -> continuous turret angle ----
        void readEncoder(double dt) {
            rawEncDeg = enc.getVoltage() / ENCODER_MAX_VOLTS * 360.0;
            double ratio = ENCODER_REVS_PER_TURRET_REV;

            if (!initialized) {
                // The absolute reading fixes the turret angle modulo (360 / ratio).
                // Pick the candidate inside the travel range closest to INIT_HINT_DEG.
                double base = wrap360(rawEncDeg - encZero()) * encSign() / ratio;
                double period = 360.0 / ratio;
                double best = Double.NaN, bestDist = Double.MAX_VALUE;
                for (int k = -10; k <= 10; k++) {
                    double c = base + k * period;
                    if (c < minDeg() - 1e-6 || c > maxDeg() + 1e-6) continue;
                    double d = Math.abs(c - INIT_HINT_DEG);
                    if (d < bestDist) { bestDist = d; best = c; }
                }
                angleDeg = Double.isNaN(best) ? base : best;
                lastAngleDeg = angleDeg;
                lastRawEncDeg = rawEncDeg;
                measVelDegPerSec = 0;
                initialized = true;
                return;
            }

            // Unwrap: the raw reading jumps 360 -> 0, so take the shortest delta between samples.
            // Needs the encoder to move < 180 deg per loop, which is easy at normal loop times.
            double delta = rawEncDeg - lastRawEncDeg;
            if (delta > 180) delta -= 360;
            if (delta < -180) delta += 360;
            lastRawEncDeg = rawEncDeg;

            angleDeg += encSign() * delta / ratio;

            double rawVel = (angleDeg - lastAngleDeg) / dt;
            lastAngleDeg = angleDeg;
            measVelDegPerSec += VEL_FILTER_ALPHA * (rawVel - measVelDegPerSec); // low-pass
        }

        // ---- BRANCH CHOICE: which "lap" of the target angle to go to ----
        // The same direction can be reached as desired + 360*k. Keep the ones inside [min, max],
        // then take the CLOSEST to where we are. Shortest legal path, never past a limit, and it only
        // unwinds (the long way around) when the target crosses the seam on the outer side.
        double chooseBranch(double desired) {
            double lo = minDeg(), hi = maxDeg();
            double best = Double.NaN, bestDist = Double.MAX_VALUE;
            for (int k = -3; k <= 3; k++) {
                double c = desired + 360.0 * k;
                if (c < lo - 1e-6 || c > hi + 1e-6) continue;
                double d = Math.abs(c - angleDeg);
                if (d < bestDist) { bestDist = d; best = c; }
            }
            if (Double.isNaN(best)) {
                best = Math.max(lo, Math.min(hi, desired)); // only if OVERLAP_DEG < 0
            }
            return best;
        }

        // ---- MAIN UPDATE ----
        void update(double dt, double robotX, double robotY, double h,
                    double vx, double vy, double omega, double tx, double ty) {

            readEncoder(dt);

            // 1. Where is THIS turret on the field? (robot offset rotated by heading)
            double cosH = Math.cos(h), sinH = Math.sin(h);
            double rx = mountX() * cosH - mountY() * sinH;
            double ry = mountX() * sinH + mountY() * cosH;

            // 2. Vector from this turret to the target, and its field-frame bearing
            double dx = tx - (robotX + rx);
            double dy = ty - (robotY + ry);
            double fieldBearing = Math.atan2(dy, dx);

            // Turret angle is relative to the robot, so subtract the robot heading.
            desiredDeg = Math.toDegrees(wrapPi(fieldBearing - h));

            // 3. Velocity feedforward: how fast must the turret turn to stay on target?
            // Pivot velocity = robot velocity + (omega x offset).
            double vxm = vx - omega * ry;
            double vym = vy + omega * rx;
            double r2 = Math.max(dx * dx + dy * dy, 1.0);          // avoid divide by 0 on top of the target
            double bearingRate = (dy * vxm - dx * vym) / r2;       // rad/s, field frame
            ffVelDeg = Math.toDegrees(bearingRate - omega);        // robot rotation also turns the turret

            // 4. Branch + error
            cmdDeg = chooseBranch(desiredDeg);
            errDeg = cmdDeg - angleDeg;

            // 5. Control: KS + KP + KD (on velocity error) + KV (feedforward) + KI
            if (Math.abs(errDeg) < I_ZONE_DEG) {
                integral += errDeg * dt;
                double iMax = (KI > 1e-9) ? I_MAX_POWER / KI : 0;
                integral = Math.max(-iMax, Math.min(iMax, integral));
            } else {
                integral = 0;
            }

            double power = KP * errDeg
                    + KD * (ffVelDeg - measVelDegPerSec)
                    + KV * ffVelDeg
                    + KI * integral;
            if (Math.abs(errDeg) > DEADBAND_DEG) {
                power += KS * Math.signum(errDeg);
            }
            power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

            // 6. Soft limits: slow to zero when pushing INTO a limit
            double lo = minDeg(), hi = maxDeg();
            if (power > 0 && angleDeg > hi - SOFT_LIMIT_MARGIN_DEG) {
                power *= Math.max(0, (hi - angleDeg) / SOFT_LIMIT_MARGIN_DEG);
            } else if (power < 0 && angleDeg < lo + SOFT_LIMIT_MARGIN_DEG) {
                power *= Math.max(0, (angleDeg - lo) / SOFT_LIMIT_MARGIN_DEG);
            }

            lastPower = power;
            servo.setPower(pwrSign() * power);
        }

        void addTelemetry(org.firstinspires.ftc.robotcore.external.Telemetry t) {
            t.addData("rawEncDeg", "%.1f", rawEncDeg);
            t.addData("angle / cmd", "%.1f / %.1f", angleDeg, cmdDeg);
            t.addData("error", "%.2f deg%s", errDeg, Math.abs(errDeg) < ON_TARGET_TOL_DEG ? "  ON TARGET" : "");
            t.addData("limits", "%.0f to %.0f", minDeg(), maxDeg());
            t.addData("power", "%.2f", lastPower);
        }
    }

    // ---------------- small helpers ----------------
    /** Wrap radians to (-pi, pi]. */
    private static double wrapPi(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }

    /** Wrap degrees to [0, 360). */
    private static double wrap360(double a) {
        a %= 360.0;
        return a < 0 ? a + 360.0 : a;
    }
}