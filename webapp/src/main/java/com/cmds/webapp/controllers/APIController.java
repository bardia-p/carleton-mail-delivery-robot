package com.cmds.webapp.controllers;

import com.cmds.webapp.aspect.NeedsLogin;
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
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;

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

    @Autowired
    private SuperuserRepository superuserRepo;

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
    @NeedsLogin(type="Delivery")
    public Delivery createDelivery(HttpServletRequest request) throws IOException {
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
        if (appUser == null) return null;
        for (Robot r: robots) {
            if (Objects.equals(r.getStatus(), Robot.RobotStatus.IDLE)) {
                r.addTrip(delivery);
                robot = r;
                break;
            }
        }
        if (robot == null) {
            System.out.println("No free robot, adding to a robot's queue");
            robot = robots.stream().min(Comparator.comparingInt(r->r.getListTrips().size())).orElse(null);
            if (robot == null) {
                return null;
            }
        }
        appUser.setCurrentDelivery(delivery);
        delivery.setAssignedRobot(robot);
        deliveryRepo.save(delivery);
        System.out.println(delivery);
        return delivery;
    }

    /**
     * Post mapping for creating a delivery.
     * @param request An HttpServletRequest request.
     * @return Valid delivery if successful, null otherwise.
     * @throws IOException
     */
    @PostMapping("/createRobot")
    @NeedsLogin(type="int")
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
     * API Call to log in a user by verifying that it exists in the userRespository
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
     * Handle the update of the status of a delivery based on its ID.
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
        System.out.println(currDelivery);
        if (currDelivery == null) return 400;

        currDelivery.addStatus(status);
        deliveryRepo.save(currDelivery);

        if (status.equals("COMPLETE")){
            Robot currRobot = currDelivery.getAssignedRobot();
            currRobot.removeTrip(currDelivery.getDeliveryId());
            robotRepo.save(currRobot);
            deliveryRepo.delete(currDelivery);
            return 200;
        }

        return 200;
    }

    /**
     * Get mapping for delivery status.
     * @param id Delivery id.
     * @return Delivery from the id.
     * @throws IOException
     */
    @GetMapping("/getDeliveryStatus/{id}")
    public String getDeliveryStatus(@PathVariable("id") String id) throws IOException, JSONException {
        System.out.println("getDeliveryStatus()");
        Delivery delivery = deliveryRepo.findById(Long.valueOf(id)).orElse(null);

        JSONObject res = new JSONObject();

        if (delivery == null){
            res.put("statuses",  new JSONObject());
        } else {
            JSONObject statuses = new JSONObject();

            for (int i = 0; i < delivery.getStatuses().size(); i++){
                statuses.put(String.valueOf(i + 1), delivery.getStatuses().get(i));
            }
            res.put("statuses", statuses);
        }

        // Extract specific data from the parsed JSON
        return res.toString();
    }

    /**
     * API Call for getting a JSON formatted object of all deliveries for a given Robot ID.
     * @param id String Robot ID.
     * @return A Json formatted list of deliveries for a particular robot.
     * @throws JSONException
     */
    @GetMapping("getRobotDeliveries/{id}")
    public String getRobotDeliveries(@PathVariable("id") String id) throws JSONException  {
        System.out.println("getRobotDeliveries() API");

        Robot robot = robotRepo.findByName(id);
        JSONObject robotObject = new JSONObject();
        if (robot != null) {
            JSONObject deliveryObject = new JSONObject();
            for (Delivery d: robot.getListTrips()) {
                if (d.getStatuses().get(d.getStatuses().size() - 1).equals("NEW")) {
                    deliveryObject.put("sourceDest", d.getStartingDest());
                    deliveryObject.put("finalDest", d.getFinalDest());
                    robotObject.put(d.getDeliveryId().toString(), deliveryObject);
                }
            }
        } else {
            System.out.println("No robot found of ID " + id);
            return "";
        }
        System.out.println(robotObject.toString());
        return robotObject.toString();
    }

    /**
     * Post mapping for creating a superuser.
     * @param request An HttpServletRequest request.
     * @return 200 if successful, 401 otherwise.
     * @throws IOException
     */
    @PostMapping("/createSuperuser")
    public int createSuperuser(HttpServletRequest request) throws IOException {
        System.out.println("createSuperuser() API");
        String jsonData = this.JSONBuilder(request);
        ObjectMapper objectMapper = new ObjectMapper();
        HashMap<String, String> userData = objectMapper.readValue(jsonData, new TypeReference<HashMap<String, String>>() {});
        System.out.println(userData);
        String username = userData.get("username");
        String password = userData.get("password");
        for (Superuser superuser: superuserRepo.findAll()){
            if (superuser.getUsername().equals(username)){
                return 401;
            }
        }
        Superuser superuser = new Superuser(username, password);
        superuserRepo.save(superuser);
        System.out.println(superuser);
        return 200;
    }

    /**
     * Get mapping for killing a robot.
     * @param id String robot id.
     * @return A JSON parsed string, with a kill value that is true or false for a given robot.
     * @throws IOException
     * @throws JSONException
     */
    @GetMapping("/killRobot/{id}")
    public String killRobot(@PathVariable("id") String id) throws IOException, JSONException {
        System.out.println("killRobot() API:");
        Robot robot = robotRepo.findByName(id);
        JSONObject robotKillObject = new JSONObject();
        if (robot != null) {
            robotKillObject.put("kill", robot.isShouldDie());
        } else {
            System.out.println("No robot found of ID " + id);
            return "";
        }
        return robotKillObject.toString();
    }

    /**
     * Post mapping for removing a user.
     * @param request
     * @return 200 if successful, 401 otherwise.
     * @throws IOException
     * @throws JSONException
     */
    @PostMapping("/removeUser")
    public int removeUser(HttpServletRequest request) throws IOException, JSONException {
        System.out.println("removeUser() API");
        String jsonData = this.JSONBuilder(request);
        ObjectMapper objectMapper = new ObjectMapper();
        HashMap<String, String> userData = objectMapper.readValue(jsonData, new TypeReference<HashMap<String, String>>() {});
        System.out.println(userData);
        String username = userData.get("username");
        for (AppUser user: userRepo.findAll()){
            if (user.getUsername().equals(username)){
                userRepo.delete(user);
                return 200;
            }
        }
        System.out.println("User to remove not found");
        return 401;
    }

}
