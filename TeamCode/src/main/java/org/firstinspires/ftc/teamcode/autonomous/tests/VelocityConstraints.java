package org.firstinspires.ftc.teamcode.autonomous.tests;

public class VelocityConstraints {

    // TODO get deceleration at zero power
    //  and acceleration at full power and then create linear equation
    // that can convert power to acceleration

    // what about if you are going 60inches per second?
    // and you apply 0.7 you would slow down... not accelerate
    //

    // customization used linear equation when going to hang first specimen
    // vf = vi + a * t

    // acceleration = (targetVelocity - currentVelocity) * kAccel;
    // kA = max Acceleration / max velocity

    // power = targetVelocity / maxVelocity + k

    // power = loop(h +k) i = n kV * velocity + kA * acceleration + kStatic;

    // minDeceleration
    // maxVelocity
    // maxAcceleration
    // power = TargetVel / maxVelocity
    // - (targetAcceleration-currentAcceleration) / maxAcceleration
    // + (targetVel - predictedVel) / maxVelocity

    // vf = vi + a * t
    // FAVOURITE RN
    // there is no targetVelocity in here
    // power = (currentVelocity + maxAcceleration * deltaTime) / maxVelocity + 0.1
    // 20, 60 = 65 / 60

    // targetVelocity = currentVelocity + currentAcceleration * delta
    // targetVelocity = x + currentAcceleration * delta

    // nextVelocity = currentVelocity + currentAcceleration * delta
    // only dampen if nextVelocity > targetVelocity



    // 0 + 0.1 / (60)

//                double changeInVelocity = targetVelocity - currentVelocity;
//                double maxChangeInVelocity = maxAcceleration * deltaTime;

    // 30 / 0.1 = 300

    // double power = (targetVelocity - currentVelocity)
    // / maxAcceleration (+velocity per second);
    //
    // 30 - 0 / 10 = 3

    // double power = changeInVelocity / maxChangeInVelocity;
//
//                if (Math.abs(velocityDelta) > maxDeltaV) {
//                    velocityDelta = Math.signum(velocityDelta) * maxDeltaV;
//                }
//
//                double newVelocity = currentVelocity + velocityDelta;
//                double power = newVelocity / maxVelocity;


//                power =
//                    (targetVelocity / maxVelocity) +
//                        ((targetVelocity - predictedVelocity) / maxVelocity) +
//                        ((currentAcceleration - targetAcceleration) / maxAcceleration)
// 0 - 10

    // power = (1 / maxVelocity) * TargetVelocity +
    // + (maxAcceleration-currentAcceleration) * (1 / maxVelocity)
    // + (targetVel - predictedVel) * (1 / maxVelocity)
    // + 0.1
    // decelerating?

    // .5 + .2 + .5

    // predictedVel = currentVel + currentAcceleration * deltaTime
    // targetVel = targetVel + deceleration * deltaTime
    // (targetVel - predictedVel) * (1/60)

    // f(power, velocity) = acceleration
    // dv/dt = a

    // solve for power and then multiply by targetAcceleration-currentAcceleration
    //
    // feedforward
    // f(targetVelocity, maxVelocity) = powerForTargetVelocity
    //

}
