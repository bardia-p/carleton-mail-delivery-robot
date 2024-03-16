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
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.autoconfigure.web.servlet.AutoConfigureMockMvc;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.test.web.servlet.MockMvc;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.get;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.content;

/**
 * Tests for the GET mappings in PageController.
 */
@SpringBootTest
@AutoConfigureMockMvc
public class PageControllerTests {

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
     * Test for the home page mapping.
     * @throws Exception
     */
    @Test
        public void testHomePageMapping() throws Exception {
            System.out.println();
            System.out.println("------------------------------");
            System.out.println("TESTING: testHomePageMapping()");
            System.out.println();
            System.out.println("Mocking get page '/', expecting to retrieve an HTML page");
            String content = this.mockMvc.perform(get("/"))
                    .andExpect(status().isOk())
                    .andExpect(content().contentType("text/html;charset=UTF-8"))
                    .andReturn().getResponse().getContentAsString();

            // extract the title
            System.out.println("Parsing the title of the page");
            Pattern pattern = Pattern.compile("<title>(.*?)</title>");
            Matcher matcher = pattern.matcher(content);

            // Find the title using the regex pattern
            String title = "";
            if (matcher.find()) {
                title = matcher.group(1);
            }
            System.out.println("Expects title: Mail Delivery Robot, Actual: " + title);
            // assert the title equals to the home page title
            assertEquals("Mail Delivery Robot - Home", title);
            System.out.println("---------------------------------");
        }

    /**
     * Test for the create delivery mapping.
     * @throws Exception
     */
    @Test
    public void testCreateDelivery() throws Exception {
        System.out.println();
        System.out.println("------------------------------");
        System.out.println("TESTING: testCreateDeliveryPageMapping()");
        System.out.println();
        AppUser user = new AppUser("TestCreateSurveyMappingUser", "test1");
        userRepo.save(user);
        Cookie cookie = new Cookie("username", user.getUsername());
        System.out.println("Mocking get page '/createDelivery', expecting to retrieve an HTML page");
        String content = this.mockMvc.perform(get("/createDelivery")
                        .cookie(cookie))
                .andExpect(status().isOk())
                .andExpect(content().contentType("text/html;charset=UTF-8"))
                .andReturn().getResponse().getContentAsString();

        // extract the title
        System.out.println("Parsing the title of the page");
        Pattern pattern = Pattern.compile("<title>(.*?)</title>");
        Matcher matcher = pattern.matcher(content);

        // Find the title using the regex pattern
        String title = "";
        if (matcher.find()) {
            title = matcher.group(1);
        }

        System.out.println("Expects title: Mail Delivery Robot - Create Delivery, Actual: " + title);
        // assert the title equals to the create survey page title
        assertEquals("Mail Delivery Robot - Create Delivery", title);
        System.out.println("------------------------------");
    }


    /**
     * Test for the manage Robots mapping.
     * @throws Exception
     */
    @Test
    public void testManageRobots() throws Exception {
        System.out.println();
        System.out.println("------------------------------");
        System.out.println("TESTING: testManageRobotsPageMapping()");
        System.out.println();
        Superuser user = new Superuser("TestManageRobotsUser", "test1");
        superuserRepo.save(user);
        Cookie cookie = new Cookie("username", user.getUsername());
        System.out.println("Mocking get page '/manageRobots', expecting to retrieve an HTML page");
        String content = this.mockMvc.perform(get("/manageRobots")
                        .cookie(cookie))
                .andExpect(status().isOk())
                .andExpect(content().contentType("text/html;charset=UTF-8"))
                .andReturn().getResponse().getContentAsString();

        // extract the title
        System.out.println("Parsing the title of the page");
        Pattern pattern = Pattern.compile("<title>(.*?)</title>");
        Matcher matcher = pattern.matcher(content);

        // Find the title using the regex pattern
        String title = "";
        if (matcher.find()) {
            title = matcher.group(1);
        }

        System.out.println("Expects title: Mail Delivery Robot - Manage Robots, Actual: " + title);
        // assert the title equals to the create survey page title
        assertEquals("Mail Delivery Robot - Manage Robots", title);
        System.out.println("------------------------------");
    }

    /**
     * Test for the remove user mapping.
     * @throws Exception
     */
    @Test
    public void testRemoveUser() throws Exception {
        System.out.println();
        System.out.println("------------------------------");
        System.out.println("TESTING: testRemoveUserPageMapping()");
        System.out.println();
        Superuser user = new Superuser("TestRemoveUserUser", "test1");
        superuserRepo.save(user);
        Cookie cookie = new Cookie("username", user.getUsername());
        System.out.println("Mocking get page '/removeUser', expecting to retrieve an HTML page");
        String content = this.mockMvc.perform(get("/removeUser")
                        .cookie(cookie))
                .andExpect(status().isOk())
                .andExpect(content().contentType("text/html;charset=UTF-8"))
                .andReturn().getResponse().getContentAsString();

        // extract the title
        System.out.println("Parsing the title of the page");
        Pattern pattern = Pattern.compile("<title>(.*?)</title>");
        Matcher matcher = pattern.matcher(content);

        // Find the title using the regex pattern
        String title = "";
        if (matcher.find()) {
            title = matcher.group(1);
        }

        System.out.println("Expects title: Mail Delivery Robot - Remove User, Actual: " + title);
        // assert the title equals to the create survey page title
        assertEquals("Mail Delivery Robot - Remove User", title);
        System.out.println("------------------------------");
    }

    /**
     * Test for the get admin page mapping.
     * @throws Exception
     */
    @Test
    public void testGetAdminPage() throws Exception {
        System.out.println();
        System.out.println("------------------------------");
        System.out.println("TESTING: testGetAdminPage()");
        System.out.println();
        Superuser user = new Superuser("TestGetAdminPage", "test1");
        superuserRepo.save(user);
        Cookie cookie = new Cookie("username", user.getUsername());
        System.out.println("Mocking get page '/admin', expecting to retrieve an HTML page");
        String content = this.mockMvc.perform(get("/admin")
                        .cookie(cookie))
                .andExpect(status().isOk())
                .andExpect(content().contentType("text/html;charset=UTF-8"))
                .andReturn().getResponse().getContentAsString();

        // extract the title
        System.out.println("Parsing the title of the page");
        Pattern pattern = Pattern.compile("<title>(.*?)</title>");
        Matcher matcher = pattern.matcher(content);

        // Find the title using the regex pattern
        String title = "";
        if (matcher.find()) {
            title = matcher.group(1);
        }

        System.out.println("Expects title: Mail Delivery Robot - Admin, Actual: " + title);
        // assert the title equals to the create survey page title
        assertEquals("Mail Delivery Robot - Admin", title);
        System.out.println("------------------------------");
    }

    /**
     * Test for the get login page mapping.
     * @throws Exception
     */
    @Test
    public void testGetLoginPage() throws Exception {
        System.out.println();
        System.out.println("------------------------------");
        System.out.println("TESTING: testGetLoginPage()");
        System.out.println();
        Superuser user = new Superuser("TestGetLoginPage", "test1");
        superuserRepo.save(user);
        Cookie cookie = new Cookie("username", user.getUsername());
        System.out.println("Mocking get page '/login', expecting to retrieve an HTML page");
        String content = this.mockMvc.perform(get("/login")
                        .cookie(cookie))
                .andExpect(status().isOk())
                .andExpect(content().contentType("text/html;charset=UTF-8"))
                .andReturn().getResponse().getContentAsString();

        // extract the title
        System.out.println("Parsing the title of the page");
        Pattern pattern = Pattern.compile("<title>(.*?)</title>");
        Matcher matcher = pattern.matcher(content);

        // Find the title using the regex pattern
        String title = "";
        if (matcher.find()) {
            title = matcher.group(1);
        }

        System.out.println("Expects title: Mail Delivery Robot - Login, Actual: " + title);
        // assert the title equals to the create survey page title
        assertEquals("Mail Delivery Robot - Login", title);
        System.out.println("------------------------------");
    }

    /**
     * Test for the get register page mapping.
     * @throws Exception
     */
    @Test
    public void testGetRegisterPage() throws Exception {
        System.out.println();
        System.out.println("------------------------------");
        System.out.println("TESTING: testGetRegisterPage()");
        System.out.println();
        Superuser user = new Superuser("TestGetRegisterPage", "test1");
        superuserRepo.save(user);
        Cookie cookie = new Cookie("username", user.getUsername());
        System.out.println("Mocking get page '/registerUser', expecting to retrieve an HTML page");
        String content = this.mockMvc.perform(get("/registerUser")
                        .cookie(cookie))
                .andExpect(status().isOk())
                .andExpect(content().contentType("text/html;charset=UTF-8"))
                .andReturn().getResponse().getContentAsString();

        // extract the title
        System.out.println("Parsing the title of the page");
        Pattern pattern = Pattern.compile("<title>(.*?)</title>");
        Matcher matcher = pattern.matcher(content);

        // Find the title using the regex pattern
        String title = "";
        if (matcher.find()) {
            title = matcher.group(1);
        }

        System.out.println("Expects title: Mail Delivery Robot - Register User, Actual: " + title);
        // assert the title equals to the create survey page title
        assertEquals("Mail Delivery Robot - Register User", title);
        System.out.println("------------------------------");
    }

    /**
     * Test for the get register robot page mapping.
     * @throws Exception
     */
    @Test
    public void testGetRegisterRobotPage() throws Exception {
        System.out.println();
        System.out.println("------------------------------");
        System.out.println("TESTING: testGetRegisterRobotPage()");
        System.out.println();
        Superuser user = new Superuser("TestGetRegisterRobotPage", "test1");
        superuserRepo.save(user);
        Cookie cookie = new Cookie("username", user.getUsername());
        System.out.println("Mocking get page '/registerRobot', expecting to retrieve an HTML page");
        String content = this.mockMvc.perform(get("/registerRobot")
                        .cookie(cookie))
                .andExpect(status().isOk())
                .andExpect(content().contentType("text/html;charset=UTF-8"))
                .andReturn().getResponse().getContentAsString();

        // extract the title
        System.out.println("Parsing the title of the page");
        Pattern pattern = Pattern.compile("<title>(.*?)</title>");
        Matcher matcher = pattern.matcher(content);

        // Find the title using the regex pattern
        String title = "";
        if (matcher.find()) {
            title = matcher.group(1);
        }

        System.out.println("Expects title: Mail Delivery Robot - Register Robot, Actual: " + title);
        // assert the title equals to the create survey page title
        assertEquals("Mail Delivery Robot - Register Robot", title);
        System.out.println("------------------------------");
    }

    /**
     * Test for the get register superuser page mapping.
     * @throws Exception
     */
    @Test
    public void testGetRegisterSuperuser() throws Exception {
        System.out.println();
        System.out.println("------------------------------");
        System.out.println("TESTING: testGetRegisterSuperuserPage()");
        System.out.println();
        Superuser user = new Superuser("TestGetRegisterSuperuserPage", "test1");
        superuserRepo.save(user);
        Cookie cookie = new Cookie("username", user.getUsername());
        System.out.println("Mocking get page '/registerSuperuser', expecting to retrieve an HTML page");
        String content = this.mockMvc.perform(get("/registerSuperuser")
                        .cookie(cookie))
                .andExpect(status().isOk())
                .andExpect(content().contentType("text/html;charset=UTF-8"))
                .andReturn().getResponse().getContentAsString();

        // extract the title
        System.out.println("Parsing the title of the page");
        Pattern pattern = Pattern.compile("<title>(.*?)</title>");
        Matcher matcher = pattern.matcher(content);

        // Find the title using the regex pattern
        String title = "";
        if (matcher.find()) {
            title = matcher.group(1);
        }

        System.out.println("Expects title: Mail Delivery Robot - Register Superuser, Actual: " + title);
        // assert the title equals to the create survey page title
        assertEquals("Mail Delivery Robot - Register Superuser", title);
        System.out.println("------------------------------");
    }

    /**
     * Test for the get delivery status page mapping.
     * @throws Exception
     */
    @Test
    public void testGetStatusPage() throws Exception {
        System.out.println();
        System.out.println("------------------------------");
        System.out.println("TESTING: testGetStatusPage()");
        System.out.println();
        Superuser user = new Superuser("TestGetStatusPage", "test1");
        superuserRepo.save(user);
        Delivery delivery = new Delivery("test", "test");
        deliveryRepo.save(delivery);
        Cookie cookie = new Cookie("username", user.getUsername());
        System.out.println("Mocking get page '/getStatus/{id}', expecting to retrieve an HTML page");
        String content = this.mockMvc.perform(get("/status/1")
                        .cookie(cookie))
                .andExpect(status().isOk())
                .andExpect(content().contentType("text/html;charset=UTF-8"))
                .andReturn().getResponse().getContentAsString();

        // extract the title
        System.out.println("Parsing the title of the page");
        Pattern pattern = Pattern.compile("<title>(.*?)</title>");
        Matcher matcher = pattern.matcher(content);

        // Find the title using the regex pattern
        String title = "";
        if (matcher.find()) {
            title = matcher.group(1);
        }

        System.out.println("Expects title: Mail Delivery Robot - Status, Actual: " + title);
        // assert the title equals to the create survey page title
        assertEquals("Mail Delivery Robot - Status", title);
        System.out.println("------------------------------");
    }

    /**
     * Test for the get robot page mapping.
     * @throws Exception
     */
    @Test
    public void testGetRobotPage() throws Exception {
        System.out.println();
        System.out.println("------------------------------");
        System.out.println("TESTING: testGetRobotPage()");
        System.out.println();
        Superuser user = new Superuser("TestGetStatusPage", "test1");
        superuserRepo.save(user);
        Robot robot = new Robot("Test");
        robotRepo.save(robot);
        Cookie cookie = new Cookie("username", user.getUsername());
        System.out.println("Mocking get page '/getRobot/{id}', expecting to retrieve an HTML page");
        String content = this.mockMvc.perform(get("/robot/Test")
                        .cookie(cookie))
                .andExpect(status().isOk())
                .andExpect(content().contentType("text/html;charset=UTF-8"))
                .andReturn().getResponse().getContentAsString();

        // extract the title
        System.out.println("Parsing the title of the page");
        Pattern pattern = Pattern.compile("<title>(.*?)</title>");
        Matcher matcher = pattern.matcher(content);

        // Find the title using the regex pattern
        String title = "";
        if (matcher.find()) {
            title = matcher.group(1);
        }

        System.out.println("Expects title: Mail Delivery Robot - Robot Status, Actual: " + title);
        // assert the title equals to the create survey page title
        assertEquals("Mail Delivery Robot - Robot Status", title);
        System.out.println("------------------------------");
    }

}
