package com.cmds.webapp;

import com.cmds.webapp.models.AppUser;
import com.cmds.webapp.models.Delivery;
import com.cmds.webapp.models.Superuser;
import com.cmds.webapp.repos.DeliveryRepository;
import com.cmds.webapp.repos.SuperuserRepository;
import com.cmds.webapp.repos.UserRepository;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.context.SpringBootTest;

import java.util.Optional;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

/**
 * Tests for the Superuser model.
 */
@SpringBootTest
public class SuperuserTests {

    @Autowired
    private UserRepository userRepo;

    @Autowired
    private DeliveryRepository deliveryRepo;

    @Autowired
    private SuperuserRepository superuserRepo;

    /**
     * Test for setting the current delivery of a superuser.
     */
    @Test
    public void testSetCurrentDelivery() {
        Superuser user = new Superuser("test2", "test");
        Delivery delivery = new Delivery();
        superuserRepo.save(user);
        deliveryRepo.save(delivery);
        assertNull(user.getCurrentDelivery());
        user.setCurrentDelivery(delivery);
        assertEquals(user.getCurrentDelivery(), delivery);
    }

}
