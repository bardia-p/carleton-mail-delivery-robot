package com.cmds.webapp;

import com.cmds.webapp.models.AppUser;
import com.cmds.webapp.models.Delivery;
import com.cmds.webapp.models.Robot;
import com.cmds.webapp.repos.DeliveryRepository;
import com.cmds.webapp.repos.RobotRepository;
import com.cmds.webapp.repos.UserRepository;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.context.SpringBootTest;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for the Delivery model.
 */
@SpringBootTest
public class DeliveryTests {

    @Autowired
    private DeliveryRepository deliveryRepo;

    @Autowired
    private RobotRepository robotRepo;

    /**
     * Test for setting a delivery's assigned robot.
     */
    @Test
    public void testSetAssignedRobot() {
        Delivery delivery = new Delivery("Loeb", "Nicol");
        Robot robot = new Robot("CREATE2-3");
        robotRepo.save(robot);
        deliveryRepo.save(delivery);
        assertNull(delivery.getAssignedRobot());
        delivery.setAssignedRobot(robot);
        assertEquals(delivery.getAssignedRobot(), robot);
    }

    /**
     * Test for adding a status to a delivery object.
     */
    @Test
    public void testAddStatus() {
        Delivery delivery = new Delivery("Loeb", "Steacie");
        deliveryRepo.save(delivery);
        assertFalse(delivery.getStatuses().contains("TEST STATUS"));
        delivery.addStatus("TEST STATUS");
        assertTrue(delivery.getStatuses().contains("TEST STATUS"));

    }

    /**
     * Test for checking if a Delivery object can persist on the back end.
     */
    @Test
    public void testPersist() {
        Delivery delivery = new Delivery("TestPersistStart", "TestPersistEnd");
        deliveryRepo.save(delivery);
        Delivery retrievedDelivery = deliveryRepo.findById(delivery.getDeliveryId()).orElse(null);
        assertEquals(2, retrievedDelivery.getDeliveryId());
    }

}
