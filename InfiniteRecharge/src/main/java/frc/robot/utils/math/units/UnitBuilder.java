package frc.robot.utils.math.units;

import java.util.ArrayList;
import java.util.Arrays;

/*
 * ==================================
 * 2020 Code Review Party Information
 * 
 * Feel free to ignore this file
 * ==================================
 */
public class UnitBuilder {
    private ArrayList<BaseUnit> nums = new ArrayList<BaseUnit>();
    private ArrayList<BaseUnit> denoms = new ArrayList<BaseUnit>();
    private double coeff0 = 1;
    private String name = null;

    public UnitBuilder num(BaseUnit... nums) {
        this.nums.addAll(Arrays.asList(nums));

        return this;
    }
    
    public UnitBuilder num(Unit... nums) {
    	for (int i = 0; i < nums.length; i++) {
    		Unit num = nums[i];
    		
    		coeff0 *= num.getCoefficient();
    		
    		this.nums.addAll(num.getNumeratorList());
    		this.denoms.addAll(num.getDenominatorList());
    	}
        

        return this;
    }

    public UnitBuilder denom(BaseUnit... denoms) {
    	this.denoms.addAll(Arrays.asList(denoms));

        return this;
    }
    
    public UnitBuilder denom(Unit... denoms) {
    	for (int i = 0; i < denoms.length; i++) {
    		Unit denom = denoms[i];
    		
    		coeff0 /= denom.getCoefficient();
    		
    		this.nums.addAll(denom.getDenominatorList());
    		this.denoms.addAll(denom.getNumeratorList());
    	}
        

        return this;
    }

    public UnitBuilder coeff(double coeff0) {
        this.coeff0 *= coeff0;

        return this;
    }
    
    public UnitBuilder name(String name) {
    	this.name = name;
    	
    	return this;
    }

    public Unit make() {
        return new Unit(nums, denoms, coeff0, name);
    }
}