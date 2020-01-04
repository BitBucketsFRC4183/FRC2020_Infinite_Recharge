package frc.robot.utils.data;

import java.util.ArrayList;

/** Unordered data window */
public class DataWindow<T> {
    private ArrayList<T> data;
    private final int LENGTH;

    private int next; // next index to replace
    private boolean filled;




    public DataWindow(int length) {
        LENGTH = length;
        data   = new ArrayList<T>(LENGTH + 1); // capacity = LENGTH + 1



        next   = 0;
        filled = false;
    }





    public void reset() {
        next   = 0;
        filled = false;

        data.clear();
    }



    public void add(T elm) {
        data.set(next, elm);
        next++;

        if (next == LENGTH) {
            filled = true; // whole window is filled with data
            next = 0; // next data additional will replace the first data entry
        }
    }



    // do we have enough data?
    public boolean isFilled() {
        return filled;
    }



    public T get(int i) {
    	// change how data is accessed to make it look like you push back
    	// old data to make space for new data
    	return data.get((next + i) % LENGTH);
    }

    public int size() {
        return LENGTH;
    }





    @Override
    public String toString() {
        String ret = "";

        for (int i = 0; i < LENGTH; i++) {
            ret += "\t" + data.get(i).toString() + "\n";
        }

        return ret;
    }
}