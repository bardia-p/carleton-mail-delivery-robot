package com.cmds.webapp;

import com.cmds.webapp.models.Delivery;
import com.cmds.webapp.models.Robot;
import com.cmds.webapp.repos.DeliveryRepository;
import com.cmds.webapp.repos.RobotRepository;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.context.SpringBootTest;

import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Tests for the Robot model.
 */
@SpringBootTest
public class RobotTests {

    @Autowired
    private DeliveryRepository deliveryRepo;

    @Autowired
    private RobotRepository robotRepo;

    /**
     * Test for adding a trip to the robot.
     */
    @Test
    public void testAddTrip() {
        Delivery delivery = new Delivery();
        Robot robot = new Robot("CREATE2-1");
        robotRepo.save(robot);
        deliveryRepo.save(delivery);
        assertFalse(robot.getListTrips().contains(delivery));
        robot.addTrip(delivery);
        assertTrue(robot.getListTrips().contains(delivery));
    }

    /**
     * Test for removing a trip from the robot.
     */
    @Test
    public void testRemoveTrip() {
        Delivery delivery = new Delivery();
        Robot robot = new Robot("CREATE2-2");
        robotRepo.save(robot);
        deliveryRepo.save(delivery);
        robot.addTrip(delivery);
        assertTrue(robot.getListTrips().contains(delivery));
        robot.removeTrip(delivery.getDeliveryId());
        assertFalse(robot.getListTrips().contains(delivery));
    }

    /**
     * Test that checks if a Robot object persists on the back end.
     */
    @Test
    public void testPersist() {
        Robot robot = new Robot("TestPersistRobot");
        robotRepo.save(robot);
        Robot retrievedRobot = robotRepo.findByName("TestPersistRobot");
        assertEquals(robot.getName(), retrievedRobot.getName());
    }
}
