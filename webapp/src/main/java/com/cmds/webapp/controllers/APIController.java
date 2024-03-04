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
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import java.io.BufferedReader;
import java.io.IOException;
import java.util.HashMap;

@RestController
@RequestMapping("/api/v1")
@NoArgsConstructor
public class APIController {

    @Autowired
    private UserRepository userRepo;

    @Autowired
    private RobotRepository robotRepo;

    @Autowired
    private DeliveryRepository deliveryRepositoryRepo;

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
     * <p>API Call to login a user by verifying that it exists in the userRespository</p>
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
     * <p>API Call to log out a user by deleting their cookies</p>
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
}
