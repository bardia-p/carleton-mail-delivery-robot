package com.cmds.webapp.controllers;

import jakarta.servlet.http.HttpServletRequest;
import lombok.NoArgsConstructor;
import org.springframework.stereotype.Controller;
import org.springframework.ui.Model;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;

/**
 * Route controller for the Mail Delivery Robot web application pages.
 */
@Controller
@NoArgsConstructor
public class PageController {

    /**
     * Get mapping for the home page.
     * @param model
     * @param request
     * @return Home page mapping.
     */
    @GetMapping("/")
    public String getHomePage(Model model, HttpServletRequest request) {
        CookieController.setUsernameCookie(model, request);
        return "index";
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
        return "register";
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
