package frc.robot.utils.data.filters;



/**
 * Filters a boolean signal to only output true in the first instance of true, and
 * only return another true once the input has been through a false
 */
public class RisingEdgeFilter {
    private boolean val = false;



    public RisingEdgeFilter() {};

    public RisingEdgeFilter(boolean val) {
        this.val = val;
    }



    public boolean calculate(boolean input) {
        // if false, no signal
        if (input == false) {
            val = false;

            return false;
        } else {
            // if previous calculation already had true, the rising edge already happened
            if (val == true) {
                return false;
            // but if it was false, then we've been through at least one false
            } else {
                // now in rising edge
                val = true;

                // therefore return true
                return true;
            }
        }
    }
}