package com.cmds.webapp.controllers;

import com.cmds.webapp.models.Robot;
import com.cmds.webapp.repos.DeliveryRepository;
import com.cmds.webapp.repos.RobotRepository;
import com.cmds.webapp.repos.UserRepository;
import jakarta.servlet.http.HttpServletRequest;
import lombok.NoArgsConstructor;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Controller;
import org.springframework.ui.Model;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;

import java.util.List;
import java.util.Optional;

/**
 * Route controller for the Mail Delivery Robot web application pages.
 */
@Controller
@NoArgsConstructor
public class PageController {

    @Autowired
    private DeliveryRepository deliveryRepo;
    @Autowired
    private RobotRepository robotRepo;
    @Autowired
    private UserRepository userRepo;

    /**
     * Get mapping for the home page.
     * @param model
     * @param request
     * @return Home page mapping.
     */
    @GetMapping("/")
    public String getHomePage(Model model, HttpServletRequest request) {
        CookieController.setUsernameCookie(model, request);
        List<Robot> robotList = robotRepo.findAll();
        model.addAttribute("robots", robotList);
        return "index";
    }

    @GetMapping("/createDelivery")
    public String getDeliveryPage(Model model, HttpServletRequest request) {
        CookieController.setUsernameCookie(model, request);
        return "createDelivery";
    }

    /**
     * Get mapping for the login page.
     * @return Login page mapping.
     */
    @GetMapping("/login")
    public String getLoginPage() {
        return "login";
    }

    /**
     * Get mapping for the register page.
     * @param model
     * @param request
     * @return Register page mapping.
     */
    @GetMapping("/register")
    public String getRegisterPage(Model model, HttpServletRequest request) {
        CookieController.setUsernameCookie(model, request);
        return "registerUser";
    }

    /**
     * Get mapping for the status page.
     * @param model
     * @param request
     * @return Log page mapping.
     */
    @GetMapping("/status/{id}")
    public String getStatusPage(@PathVariable("id") String deliveryId, Model model, HttpServletRequest request) {
        CookieController.setUsernameCookie(model, request);
        return "log";
    }

    /**
     * Get mapping for the robot page.
     * @param model
     * @param request
     * @return Log page mapping.
     */
    @GetMapping("/robot/{id}")
    public String getRobotPage(@PathVariable("id") String robotId, Model model, HttpServletRequest request) {
        CookieController.setUsernameCookie(model, request);
        Robot r = robotRepo.findByName((robotId));
        model.addAttribute("robot", r);
        return "robot";
    }
    /**
     * Get mapping for the register robot page.
     * @param model
     * @param request
     * @return Login page mapping.
     */
    @GetMapping("/registerRobot")
    public String getRegisterRobotPage(Model model, HttpServletRequest request) {
        CookieController.setUsernameCookie(model, request);
        return "registerRobot";
    }

}
