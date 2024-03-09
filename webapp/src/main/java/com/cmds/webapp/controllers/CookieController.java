package com.cmds.webapp.controllers;

import jakarta.servlet.http.Cookie;
import jakarta.servlet.http.HttpServletRequest;
import org.springframework.ui.Model;

/**
 * The class in charge of maintaining all the Cookie related functions.
 */
public class CookieController {
    /**
     * Helper function to get the cookie information and add it to the model
     * @param model Model, the client Model
     * @param request An HttpServletRequest request.
     */
    public static void setUsernameCookie(Model model, HttpServletRequest request) {
        Cookie[] cookie = request.getCookies();
        model.addAttribute("username", retrieveCookie(cookie, "username"));
    }

    /**
     * Helper function to get the logged-in username from the cookie
     * @param request An HttpServletRequest request.
     * @return String
     */
    public static String getUsernameFromCookie(HttpServletRequest request){
        Cookie[] cookie = request.getCookies();
        return retrieveCookie(cookie, "username");
    }

    /**
     * Returns the cookie from the list of the cookies.
     * @param cookies the list of the cookies
     * @param name the name of the cookie.
     * @return the value of the coookie.
     */
    private static String retrieveCookie(Cookie[] cookies, String name){
        String res = null;
        if (cookies == null){
            return null;
        }
        for (Cookie c : cookies) {
            if (c.getName().equals(name) && c.getMaxAge() != 0) {
                res = c.getValue();
            }
        }
        return res;
    }
}
