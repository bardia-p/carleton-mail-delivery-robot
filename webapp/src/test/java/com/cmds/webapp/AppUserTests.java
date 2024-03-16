package com.cmds.webapp;

import com.cmds.webapp.models.AppUser;
import com.cmds.webapp.models.Delivery;
import com.cmds.webapp.repos.DeliveryRepository;
import com.cmds.webapp.repos.UserRepository;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.context.SpringBootTest;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Testing for the AppUser model.
 */
@SpringBootTest
public class AppUserTests {

    @Autowired
    private UserRepository userRepo;

    @Autowired
    private DeliveryRepository deliveryRepo;

    /**
     * Test for setting the current delivery of an AppUser.
     */
    @Test
    public void testSetCurrentDelivery() {
        AppUser user = new AppUser("test1", "test");
        Delivery delivery = new Delivery();
        userRepo.save(user);
        deliveryRepo.save(delivery);
        assertNull(user.getCurrentDelivery());
        user.setCurrentDelivery(delivery);
        assertEquals(user.getCurrentDelivery(), delivery);
    }
    @Test
    public void testPersist(){
        AppUser user = new AppUser("testPersistUser", "testPass");
        userRepo.save(user);
        AppUser retrievedUser = userRepo.findByUsername("testPersistUser").orElse(null);
        assertEquals(retrievedUser, user);

    }
}
