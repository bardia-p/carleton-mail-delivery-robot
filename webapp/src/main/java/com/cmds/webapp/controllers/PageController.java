package com.cmds.webapp.controllers;

import com.cmds.webapp.aspect.NeedsLogin;
import com.cmds.webapp.models.AppUser;
import com.cmds.webapp.models.Robot;
import com.cmds.webapp.models.Superuser;
import com.cmds.webapp.repos.DeliveryRepository;
import com.cmds.webapp.repos.RobotRepository;
import com.cmds.webapp.repos.SuperuserRepository;
import com.cmds.webapp.repos.UserRepository;
import jakarta.servlet.http.Cookie;
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
    @Autowired
    private SuperuserRepository superuserRepo;

    /**
     * Boolean method for checking whether the user from a given cookie is a superuser, for purpose of endpoint protection.
     * @param request
     * @return True if a superuser, false otherwise.
     */
    public boolean isSuperUser(HttpServletRequest request) {
        String username = CookieController.getUsernameFromCookie(request);
        for (Superuser user: superuserRepo.findAll()) {
            if (user.getUsername().equals(username))
                return true;
        }
        return false;
    }

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
        List<AppUser> appUsers = userRepo.findAll();
        String username = CookieController.getUsernameFromCookie(request);
        System.out.println(username);
        if (username != null) {
            for (AppUser user: appUsers){
                if (user.getUsername().equals(username)) {
                    model.addAttribute("currentUser", user);
                    break;
                }
            }
        }
        return "index";
    }

    /**
     * Get mapping for the manage robots page.
     * @param model
     * @param request
     * @return Redirects to the home page if not a superuser, otherwise the manageRobots page.
     */
    @GetMapping("/manageRobots")
    public String getManageRobotsPage(Model model, HttpServletRequest request) {
        CookieController.setUsernameCookie(model, request);
        if (!isSuperUser(request)) {
            return "redirect:/";
        }
        List<Robot> robotList = robotRepo.findAll();
        model.addAttribute("robots", robotList);
        return "manageRobots";
    }

    /**
     * Get mapping for the create delivery page.
     * @param model
     * @param request
     * @return The createDelivery mapping.
     */
    @GetMapping("/createDelivery")
    @NeedsLogin
    public String getDeliveryPage(Model model, HttpServletRequest request) {
        CookieController.setUsernameCookie(model, request);
        return "createDelivery";
    }

    /**
     * Get mapping for the remove user page.
     * @param model
     * @param request
     * @return Back to the home page if not an admin, otherwise the remove user page.
     */
    @GetMapping("/removeUser")
    @NeedsLogin
    public String getRemoveUserPage(Model model, HttpServletRequest request) {
        CookieController.setUsernameCookie(model, request);
        if (!isSuperUser(request)) {
            return "redirect:/";
        }
        return "removeUser";
    }
    /**
     * Get mapping for the admin page.
     * @param model
     * @param request
     * @return Back to the home page if not a superuser, otherwise the admin mapping.
     */
    @GetMapping("/admin")
    @NeedsLogin
    public String getAdminPage(Model model, HttpServletRequest request) {
        CookieController.setUsernameCookie(model, request);
        if (!isSuperUser(request)) {
            return "redirect:/";
        }
        return "admin";
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
     * Get mapping for the register user page.
     * @param model
     * @param request
     * @return Register user page mapping.
     */
    @GetMapping("/registerUser")
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
    @NeedsLogin
    public String getStatusPage(@PathVariable("id") String deliveryId, Model model, HttpServletRequest request) {
        CookieController.setUsernameCookie(model, request);
        return "log";
    }

    /**
     * Get mapping for the robot page.
     * @param model
     * @param request
     * @return Robot page mapping.
     */
    @GetMapping("/robot/{id}")
    @NeedsLogin
    public String getRobotPage(@PathVariable("id") String robotId, Model model, HttpServletRequest request) {
        CookieController.setUsernameCookie(model, request);
        if (!isSuperUser(request)) {
            return "redirect:/";
        }
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
    @NeedsLogin
    public String getRegisterRobotPage(Model model, HttpServletRequest request) {
        CookieController.setUsernameCookie(model, request);
        if (!isSuperUser(request)) {
            return "redirect:/";
        }
        return "registerRobot";
    }

    /**
     * Get mapping for the register superuser page.
     * @param model
     * @param request
     * @return Register page mapping.
     */
    @GetMapping("/registerSuperuser")
    public String getRegisterSuperuserPage(Model model, HttpServletRequest request) {
        CookieController.setUsernameCookie(model, request);
        if (!isSuperUser(request)) {
            return "redirect:/";
        }
        return "registerSuperuser";
    }

}
