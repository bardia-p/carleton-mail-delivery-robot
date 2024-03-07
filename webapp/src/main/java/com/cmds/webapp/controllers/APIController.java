package com.cmds.webapp.controllers;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import jakarta.servlet.http.Cookie;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import lombok.NoArgsConstructor;
import com.cmds.webapp.models.*;
import com.cmds.webapp.repos.*;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.*;
import org.json.*;
import java.io.BufferedReader;
import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

@RestController
@RequestMapping("/api/v1")
@NoArgsConstructor
public class APIController {

    @Autowired
    private UserRepository userRepo;

    @Autowired
    private RobotRepository robotRepo;

    @Autowired
    private DeliveryRepository deliveryRepo;

    /**
     * Method for building a JSON format from a request.
     * @param request An HttpServletRequest request.
     * @return An ObjectMapper that maps a request's parameter to a particular value in JSON format.
     * @throws IOException
     */
    private String JSONBuilder(HttpServletRequest request) throws IOException {
        // read the json sent by the client
        BufferedReader reader = request.getReader();
        // create a string format of the json from the reader
        StringBuilder jsonBuilder = new StringBuilder();
        String line;
        while ((line = reader.readLine()) != null) {
            jsonBuilder.append(line);
        }
        String jsonData = jsonBuilder.toString();
        System.out.println("JSONDATA: " + jsonData);
        return jsonData;
    }

    /**
     * Post mapping for creating a user.
     * @param request An HttpServletRequest request.
     * @return 200 if successful, 401 otherwise.
     * @throws IOException
     */
    @PostMapping("/createUser")
    public int createUser(HttpServletRequest request) throws IOException {
        System.out.println("createUser() API");
        String jsonData = this.JSONBuilder(request);
        ObjectMapper objectMapper = new ObjectMapper();
        HashMap<String, String> userData = objectMapper.readValue(jsonData, new TypeReference<HashMap<String, String>>() {});
        System.out.println(userData);
        String username = userData.get("username");
        String password = userData.get("password");
        for (AppUser appUser: userRepo.findAll()){
            if (appUser.getUsername().equals(username)){
                return 401;
            }
        }
        AppUser appUser = new AppUser(username, password);
        userRepo.save(appUser);
        System.out.println(appUser);
        return 200;
    }

    /**
     * Post mapping for creating a delivery.
     * @param request An HttpServletRequest request.
     * @return Valid delivery if successful, null otherwise.
     * @throws IOException
     */
    @PostMapping("/createDelivery")
    public int createDelivery(HttpServletRequest request) throws IOException {
        System.out.println("createDelivery() API");
        String username = CookieController.getUsernameFromCookie(request);
        String jsonData = this.JSONBuilder(request);
        ObjectMapper objectMapper = new ObjectMapper();
        List<Robot> robots = robotRepo.findAll();
        HashMap<String, String> userData = objectMapper.readValue(jsonData, new TypeReference<HashMap<String, String>>() {});
        System.out.println(userData);
        String source = userData.get("source");
        String destination = userData.get("destination");
        Delivery delivery = new Delivery(source, destination);
        AppUser appUser = userRepo.findByUsername(username).orElse(null);
        Robot robot = null;
        if (appUser == null) return 400;
        System.out.println("Hello this is a test 1");
        for (Robot r: robots) {
            if (Objects.equals(r.getStatus(), "")) {
                r.addTrip(delivery);
                robot = r;
                break;
            }
        }
        if (robot == null) {
            System.out.println("Robots are all occupied with deliveries!");
            return 400;
        }
        appUser.setCurrentDelivery(delivery);
        delivery.setAssignedRobot(robot);
        deliveryRepo.save(delivery);
        System.out.println(delivery);
        return 200;
    }

    /**
     * Post mapping for creating a delivery.
     * @param request An HttpServletRequest request.
     * @return Valid delivery if successful, null otherwise.
     * @throws IOException
     */
    @PostMapping("/createRobot")
    public int createRobot(HttpServletRequest request) throws IOException {
        System.out.println("createRobot() API");
        String jsonData = this.JSONBuilder(request);
        ObjectMapper objectMapper = new ObjectMapper();
        HashMap<String, String> userData = objectMapper.readValue(jsonData, new TypeReference<HashMap<String, String>>() {});
        System.out.println(userData);
        String robotName = userData.get("robotname");
        Robot robot = new Robot(robotName);
        robotRepo.save(robot);
        System.out.println(robot);
        return 200;
    }
    /**
     * API Call to login a user by verifying that it exists in the userRespository
     * @param request HttpServletRequest, a request from the client.
     * @return 200, if user successfully logs in, 401 if user is not authenticated properly.
     * @throws IOException
     */
    @PostMapping("/loginUser")
    public int loginUser(HttpServletRequest request, HttpServletResponse response) throws IOException{
        AppUser loggedInUser = null;
        String jsonData = this.JSONBuilder(request);
        ObjectMapper objectMapper = new ObjectMapper();
        HashMap<String, String> userData = objectMapper.readValue(jsonData, new TypeReference<HashMap<String, String>>() {
        });
        String username = userData.get("username");
        String password = userData.get("password");
        for(AppUser user : userRepo.findAll()){
            if(user.getUsername().equals(username) && user.getPassword().equals(password)){
                loggedInUser = user;
                break;
            }
        }
        if(loggedInUser == null){
            return 401; //indicates unauthorized
        }

        Cookie cookie = new Cookie( "username", loggedInUser.getUsername());
        cookie.setPath("/");
        response.addCookie(cookie);
        System.out.println("Value of loggedInUser = " + loggedInUser);
        return 200;
    }

    /**
     * API Call to log out a user by deleting their cookies
     * @param response HttpServletResponse server side response.
     * @param request HttpServletRequest, a request from the client.
     * @return 200, if the API was a success.
     * @throws IOException
     */
    @PostMapping("/logout")
    public int logoutUser(HttpServletResponse response, HttpServletRequest request) throws IOException {
        Cookie[] cookies = request.getCookies();
        if (cookies != null) {
            for (Cookie c : cookies) {
                if (c.getName().equals("username")) {
                    Cookie cRemove = new Cookie("username", "");
                    cRemove.setMaxAge(0);
                    cRemove.setPath("/");
                    response.addCookie(cRemove);
                    break;
                }
            }
        }
        return 200;
    }

    /**
     * <p>Handle the update of the status of a delivery based on its ID</p>
     * @param id The delivery ID
     * @param request
     * @return 200 if successful, otherwise 400
     * @throws IOException
     */
    @PostMapping("/updateStatus/{id}")
    public int updateStatus(@PathVariable("id") String id, HttpServletRequest request) throws IOException {
        System.out.println("Updating delivery status API()");
        String jsonData = this.JSONBuilder(request);
        ObjectMapper objectMapper = new ObjectMapper();
        HashMap<String, Object> deliveryData = objectMapper.readValue(jsonData, new TypeReference<HashMap<String, Object>>() {});
        // Extract specific data from the parsed JSON
        String status = (String) deliveryData.get("status");
        Delivery currDelivery = deliveryRepo.findById((Long.valueOf(id))).orElse(null);

        if (currDelivery == null) return 400;

        currDelivery.setStatus(status);
        deliveryRepo.save(currDelivery);

        return 200;
    }

    /**
     * Get mapping for delivery status.
     * @param id Delivery id.
     * @return Delivery from the id.
     * @throws IOException
     */
    @GetMapping("/getDeliveryStatus/{id}")
    public Delivery getDeliveryStatus(@PathVariable("id") String id) throws IOException {
        System.out.println("getDeliveryStatus()");
        // Extract specific data from the parsed JSON
        return deliveryRepo.findById(Long.valueOf(id)).orElse(null);
    }


    /**
     * Test mapping to ensure it is compatible in ROS.
     * @return test string
     * @throws IOException
     */
    @GetMapping
    public String getEndpoint() throws IOException {
        return "test";
    }

    @GetMapping("getRobotDeliveries/{id}")
    public String getSurveyQuestions(@PathVariable("id") String id, HttpServletRequest request) throws JSONException  {
        System.out.println("getRobotDeliveries() API");

        Optional<Robot> r = robotRepo.findById(Long.valueOf(id));
        JSONObject deliveriesObject = new JSONObject();
        if (r.isPresent()) {
            Robot robot = r.get();
            for (Delivery d: robot.getListTrips()) {
                deliveriesObject.put("deliveries", d);
            }
        } else {
            System.out.println("No robot found of ID " + id);
            return "";
        }
        return deliveriesObject.toString();
    }
}
