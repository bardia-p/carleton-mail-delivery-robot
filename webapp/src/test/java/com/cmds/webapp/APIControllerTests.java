package com.cmds.webapp;

import com.cmds.webapp.models.AppUser;
import com.cmds.webapp.models.Delivery;
import com.cmds.webapp.models.Robot;
import com.cmds.webapp.models.Superuser;
import com.cmds.webapp.repos.DeliveryRepository;
import com.cmds.webapp.repos.RobotRepository;
import com.cmds.webapp.repos.SuperuserRepository;
import com.cmds.webapp.repos.UserRepository;
import jakarta.servlet.http.Cookie;
import jakarta.transaction.Transactional;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.autoconfigure.web.servlet.AutoConfigureMockMvc;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.http.MediaType;
import org.springframework.test.web.servlet.MockMvc;

import static org.junit.jupiter.api.Assertions.*;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.post;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

/**
 * Test for the POST mappings in APIController.
 */
@SpringBootTest
@AutoConfigureMockMvc
@Transactional
public class APIControllerTests {

    @Autowired
    private MockMvc mockMvc;

    @Autowired
    private UserRepository userRepo;

    @Autowired
    private SuperuserRepository superuserRepo;

    @Autowired
    private DeliveryRepository deliveryRepo;

    @Autowired
    private RobotRepository robotRepo;


    /**
     * Method to test the Register User post mapping. It simply verifies that a new user was added to the repository.
     *
     * @throws Exception, exception
     */
    @Test
    public void testRegisterUser() throws Exception {
        String postData = "{\"username\":\"TestRegisterUserTestUser\",\"password\":\"sysc4806\"}";
        this.mockMvc.perform(post("/api/v1/createUser")
                        .contentType(MediaType.APPLICATION_JSON).content(postData))
                .andExpect(status().isOk());

        AppUser retrievedUser = userRepo.findByUsername("TestRegisterUserTestUser").orElse(null);
        assertNotNull(retrievedUser);
        assertEquals("TestRegisterUserTestUser", retrievedUser.getUsername());
    }

    /**
     * Method to test the Register Superuser post mapping. It simply verifies that a new superuser was added to the repository.
     *
     * @throws Exception, exception
     */
    @Test
    public void testRegisterSuperuser() throws Exception {
        String postData = "{\"username\":\"TestRegisterSuperuserTestUser\",\"password\":\"sysc4806\"}";
        this.mockMvc.perform(post("/api/v1/createSuperuser")
                        .contentType(MediaType.APPLICATION_JSON).content(postData))
                .andExpect(status().isOk());

        Superuser retrievedUser = superuserRepo.findByUsername("TestRegisterSuperuserTestUser").orElse(null);
        assertNotNull(retrievedUser);
        assertEquals("TestRegisterSuperuserTestUser", retrievedUser.getUsername());
    }

    /**
     * Method that tests the loginUser POST mapping. It tests that a user can successfully be logged in.
     * @throws Exception, exception
     */
    @Test
    public void testLoginUser() throws Exception {
        String postData = "{\"username\":\"TestLoginUserTestUser\",\"password\":\"sysc4806\"}";
        this.mockMvc.perform(post("/api/v1/createUser")
                        .contentType(MediaType.APPLICATION_JSON).content(postData))
                .andExpect(status().isOk());

        Cookie cookie = new Cookie("username", "TestLoginUserTestUser");
        // Create a survey using the POST request.
        this.mockMvc.perform(post("/api/v1/loginUser")
                        .cookie(cookie)
                        .contentType(MediaType.APPLICATION_JSON).content(postData))
                .andExpect(status().isOk());

        AppUser loggedInUser = userRepo.findByUsername("TestLoginUserTestUser").orElse(null);
        assertNotNull(loggedInUser);
        assertEquals(cookie.getValue(), loggedInUser.getUsername());
    }

    /**
     * Method to test the Create Delivery post mapping. It simply verifies that a new delivery was posted to the repository.
     *
     * @throws Exception, exception
     */
    @Test
    public void testCreateDelivery() throws Exception {
        String postData = "{\"source\":\"TestStart\",\"destination\":\"TestFinal\",\"status\":\"NEW\" }";

        Superuser user = new Superuser("APITestCreateDelivery", "password");
        this.superuserRepo.save(user);
        assertNotNull(this.superuserRepo.findByUsername("APITestCreateDelivery").orElse(null));

        Robot robot = new Robot("TestRobotCreateDelivery");
        this.robotRepo.save(robot);
        assertNotNull(this.robotRepo.findByName("TestRobotCreateDelivery"));

        Cookie userCookie = new Cookie("username", user.getUsername());
        this.mockMvc.perform(post("/api/v1/createDelivery")
                        .cookie(userCookie)
                        .contentType(MediaType.APPLICATION_JSON).content(postData))
                .andExpect(status().isOk());


        Delivery retrievedDelivery = null;
        for (Delivery d: this.deliveryRepo.findAll()) {
            if (d.getStartingDest().equals("TestStart")) {
                retrievedDelivery = d;
                break;
            }
        }
        assertNotNull(retrievedDelivery);
        assertEquals("TestStart", retrievedDelivery.getStartingDest());
    }

    /**
     * Method to test the Create Robot post mapping. It simply verifies that a new robot was posted to the repository.
     *
     * @throws Exception, exception
     */
    @Test
    public void testCreateRobot() throws Exception {
        String postData = "{\"robotname\":\"TestRobot\"}";

        Superuser user = new Superuser("APITestCreateRobot", "password");
        this.superuserRepo.save(user);
        assertNotNull(this.superuserRepo.findByUsername("APITestCreateRobot").orElse(null));

        Cookie userCookie = new Cookie("username", user.getUsername());
        this.mockMvc.perform(post("/api/v1/createRobot")
                        .cookie(userCookie)
                        .contentType(MediaType.APPLICATION_JSON).content(postData))
                .andExpect(status().isOk());

       Robot retrievedRobot = null;
        for (Robot r: this.robotRepo.findAll()) {
            if (r.getName().equals("TestRobot")) {
                retrievedRobot = r;
                break;
            }
        }
        assertNotNull(retrievedRobot);
        assertEquals("TestRobot", retrievedRobot.getName());
    }
}
