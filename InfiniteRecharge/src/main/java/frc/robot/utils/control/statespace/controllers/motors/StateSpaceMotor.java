package frc.robot.utils.control.statespace.controllers.motors;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import frc.robot.utils.control.motor.BBMotorController;

import frc.robot.utils.control.statespace.models.motors.Motor;
import frc.robot.utils.control.statespace.StateSpaceException;
import frc.robot.utils.control.statespace.controllers.StateSpaceController;
import frc.robot.utils.control.statespace.models.lti.LTIModel;
import frc.robot.utils.control.statespace.models.lti.estimators.LTIEstimatorModel;



public class StateSpaceMotor extends StateSpaceController {
    private final BBMotorController MOTOR_CONTROLLER;
    private final Motor MOTOR_TYPE;

    private DMatrixRMaj K;



    public static class MotorModel extends LTIModel {
        private final BBMotorController MOTOR_CONTROLLER;

        public MotorModel(BBMotorController motorController, Motor motorType) throws StateSpaceException {
            super(motorType.getA(), motorType.getB(), motorType.getC());

            MOTOR_CONTROLLER = motorController;
            MOTOR_CONTROLLER.setSI();
        }

        @Override
        public DMatrixRMaj getOutput() {
            return new DMatrixRMaj(
                new double[] {MOTOR_CONTROLLER.getVoltage()}
            );
        }

        @Override
        public DMatrixRMaj getState() {
            return null;
        }
    }



    public StateSpaceMotor(BBMotorController motorController, Motor motorType) throws StateSpaceException {
        super(new MotorModel(motorController, motorType));

        MOTOR_CONTROLLER = motorController;
        MOTOR_TYPE = motorType;
    }



    @Override
    protected DMatrixRMaj getInput(DMatrixRMaj state) {
        if (K != null) {
            DMatrixRMaj input = null;

            CommonOps_DDRM.mult(K, state, input);

            return input;
        }

        return new DMatrixRMaj(1, 1);
    }
}