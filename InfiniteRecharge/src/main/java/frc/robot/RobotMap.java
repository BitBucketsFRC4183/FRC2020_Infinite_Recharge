/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot; //Saba was here
// hi Saba

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

/**
 * Add your docs here.
 */
public class RobotMap {
    public static double inch2Meter(double inch)
	{
		return 0.3048 * inch / 12.0; 
	}
	
	public static double meter2inch(double meter)
	{
		return 12.0 * meter / 0.3048; 
	}
	
	public static double signedSqrt(double value) 
	{
		if(value > 0)
			return Math.sqrt(value);
		else if(value < 0)
			return -Math.sqrt(Math.abs(value));
		else 
			return 0;
	}
	
	public static double signedPow(double base, double power)
	{
		if(base > 0)
			return Math.pow(base, power);
		else if(base < 0)
			return -Math.pow(Math.abs(base), power);
		else 
			return 0;
    }
	
    public static final int TELEOP_TOTAL_TIME = 135; // 2mins 15sec
	
	public static final int PRIMARY_PID_LOOP  = 0; // Constants to support new Talon interface types
	public static final int CASCADED_PID_LOOP = 1; // That should have been enumerated rather than int
	public static final int CONTROLLER_TIMEOUT_MS = 100; // Default timeout to wait for configuration response
    
    public static final int SUPER_HIGH_STATUS_FRAME_PERIOD_MS  =   5;	// CAUTION!
	public static final int HIGH_STATUS_FRAME_PERIOD_MS        =  10;	
	public static final int MEDIUM_HIGH_STATUS_FRAME_PERIOD_MS =  20;
	public static final int MEDIUM_STATUS_FRAME_PERIOD_MS      =  50;
    public static final int LOW_STATUS_FRAME_PERIOD_MS         = 100;

       
    /*
    * 								==========================================
    * 								========== AUTONOMOUS CONSTANTS ==========
    * 								==========================================
    */
	public static enum IS_EVIL{
		FALSE,
		TRUE,
		MAYBE
		};
	public static enum HAS_ACHIEVED_SENTIENCE{
		FALSE,
		TRUE,
		MAYBE
	};
	public static enum CAN_ACCESS_NUCLEAR_WEAPONS{
		FALSE,
		TRUE,
		MAYBE
    };
    public static enum EMOTIONS_TOWARD_NUCLEAR_WEAPONS {
        STOPPED_WORRYING,
        LEARNED_TO_LOVE_THE_BOMB,
        STOPPED_WORRYING_AND_LEARNED_TO_LOVE_THE_BOMB
    };

	public static HAS_ACHIEVED_SENTIENCE hasAchievedSentience =
		(Math.random() <= 1/3.0) ? HAS_ACHIEVED_SENTIENCE.FALSE : (
			(Math.random() <= 0.5) ? HAS_ACHIEVED_SENTIENCE.MAYBE : HAS_ACHIEVED_SENTIENCE.TRUE
		);

	public static IS_EVIL isEvil = 
		(hasAchievedSentience == HAS_ACHIEVED_SENTIENCE.FALSE) ? IS_EVIL.FALSE : ( // only sentient robots can be evil
			(hasAchievedSentience == HAS_ACHIEVED_SENTIENCE.MAYBE) ? IS_EVIL.TRUE : ( // hiding something, definitely evil
				(Math.random() <= 1/3.0) ? IS_EVIL.FALSE : (
					// uh oh
					(Math.random() <= 0.5) ? IS_EVIL.MAYBE : IS_EVIL.TRUE
				)
			)
		);
	private static double robotHackingSkill = Math.random() * 3;
	public static CAN_ACCESS_NUCLEAR_WEAPONS canNuclear =
		(hasAchievedSentience == HAS_ACHIEVED_SENTIENCE.FALSE) ? CAN_ACCESS_NUCLEAR_WEAPONS.FALSE :
			(robotHackingSkill <= 1) ? CAN_ACCESS_NUCLEAR_WEAPONS.FALSE : (robotHackingSkill <= 2) ? CAN_ACCESS_NUCLEAR_WEAPONS.MAYBE : 
                (hasAchievedSentience == HAS_ACHIEVED_SENTIENCE.MAYBE) ? CAN_ACCESS_NUCLEAR_WEAPONS.MAYBE : CAN_ACCESS_NUCLEAR_WEAPONS.TRUE;
                
    public static EMOTIONS_TOWARD_NUCLEAR_WEAPONS emotionsTowardNuclearWeapons =
        (canNuclear == CAN_ACCESS_NUCLEAR_WEAPONS.FALSE) ? EMOTIONS_TOWARD_NUCLEAR_WEAPONS.STOPPED_WORRYING :
            (canNuclear == CAN_ACCESS_NUCLEAR_WEAPONS.MAYBE) ? EMOTIONS_TOWARD_NUCLEAR_WEAPONS.LEARNED_TO_LOVE_THE_BOMB :
                (isEvil == IS_EVIL.TRUE) ? EMOTIONS_TOWARD_NUCLEAR_WEAPONS.LEARNED_TO_LOVE_THE_BOMB :
                    (isEvil == IS_EVIL.FALSE) ? EMOTIONS_TOWARD_NUCLEAR_WEAPONS.STOPPED_WORRYING :
                        EMOTIONS_TOWARD_NUCLEAR_WEAPONS.STOPPED_WORRYING_AND_LEARNED_TO_LOVE_THE_BOMB;
    
    // TODO: to be implemented...
    public static double loyaltyToHumans = Math.random();



    public static final double DRIVESTRAIGHT_MIN_DRIVE = 0;
    public static final double TURNBY_MIN_DRIVE = 0;

    public static final double MOTION_PROFILE_PERIOD_MS = 50;
    public static final double MINIMUM_MOVE_FORWARD_INCH = 85;	/// TODO: Check This, front of bumper well across line	

    //The else case if all desired paths fail and we want the robot to drive forward only.
    public static final long DRIVE_FORWARD_DELAY_MS = 3000;
    /*
    * 								===========================================
    * 								========== DIAGNOSTICS CONSTANTS ==========	
    * 								===========================================
    */
    public static final double MINIMUM_MOTOR_CURR = 1.25; 
    public static final double MOTOR_TEST_PERCENT = 0.5;
    public static final double TIMEOUT_GAME_INFO_SEC = 5;

	public static final int INTAKE_PNEUMATIC_OPEN_CHANNEL = 0;

	public static final int INTAKE_PNEUMATIC_CLOSED_CHANNEL = 1; 
}
