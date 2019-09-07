/*
 * This Java source file was generated by the Gradle 'init' task.
 */
package frc.robot.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

import org.junit.Test;

import frc.robot.Presets;

import org.junit.After;
import org.junit.Before;
import org.junit.AfterClass;
import org.junit.BeforeClass;
import org.junit.Ignore;



public class DeadzoneFilterTest {

    @Before public void setup(){
        System.out.println("No setup");
    }

    @Test public void testSample() {
        assert(0==Drivetrain.deadzoneFilter(0.004));
        assert(1==Drivetrain.deadzoneFilter(1));
        assert(Drivetrain.deadzoneFilter(0.75) > 0.72 && Drivetrain.deadzoneFilter(0.75) < 0.74);


    }

    @After public void cleanup(){
        System.out.println("no cleanup");
    }
}

