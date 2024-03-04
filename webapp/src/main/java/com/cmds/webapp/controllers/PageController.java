package com.cmds.webapp.controllers;

import jakarta.servlet.http.HttpServletRequest;
import lombok.NoArgsConstructor;
import org.springframework.stereotype.Controller;
import org.springframework.ui.Model;
import org.springframework.web.bind.annotation.GetMapping;

/**
 * Route controller for Opinion Owl pages
 */
@Controller
@NoArgsConstructor
public class PageController {

    @GetMapping("/")
    public String getHomePage(Model model, HttpServletRequest request) {
        CookieController.setUsernameCookie(model, request);
        return "index";
    }

    @GetMapping("/login")
    public String getLoginPage(Model model, HttpServletRequest request) {
        return "login";
    }

    @GetMapping("/register")
    public String getRegisterPage(Model model, HttpServletRequest request) {
        return "register";
    }

    @GetMapping("/status")
    public String getStatusPage(Model model, HttpServletRequest request) {

        CookieController.setUsernameCookie(model, request);
        return "log";
    }

}
