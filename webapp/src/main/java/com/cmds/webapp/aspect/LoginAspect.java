package com.cmds.webapp.aspect;

import com.cmds.webapp.controllers.CookieController;
import jakarta.servlet.http.HttpServletRequest;
import org.aspectj.lang.ProceedingJoinPoint;
import org.aspectj.lang.annotation.Around;
import org.aspectj.lang.annotation.Aspect;
import org.aspectj.lang.annotation.Pointcut;
import org.springframework.stereotype.Component;

/**
 * Class LoginAspect for defining an aspect for all login checks within APIController and PageController.
 */
@Aspect
@Component
public class LoginAspect {

    /**
     * Method for defining the annotation needsLogin.
     * @param needsLogin NeedsLogin object
     */
    @Pointcut("@annotation(needsLogin)")
    public void callAt(NeedsLogin needsLogin) {
    }

    /**
     * Method for checking the type of the NeedsLogin annotation and proceeding with the actual login check through the aspect.
     * @param pjp A ProceedingJoinPoint pjp.
     * @param needsLogin A NeedsLogin needsLogin.
     * @return The type of the aspect annotation
     * @throws Throwable An error
     */
    @Around(value = "callAt(needsLogin)", argNames = "pjp,needsLogin")
    public Object around(ProceedingJoinPoint pjp, NeedsLogin needsLogin) throws Throwable {
        Object[] args = pjp.getArgs();
        HttpServletRequest request = null;
        for (Object arg : args) {
            if (arg instanceof HttpServletRequest) {
                request = (HttpServletRequest) arg;
            }
        }
        if (request == null) {
            return getReturnType(needsLogin);
        }
        String res = CookieController.getUsernameFromCookie(request);
        if (res == null) {
            return getReturnType(needsLogin);
        }
        return pjp.proceed();
    }

    /**
     * Method for getting the return type for the around method.
     * @param needsLogin A needsLogin object
     * @return Html, string or int type
     */
    public Object getReturnType(NeedsLogin needsLogin) {
        return switch (needsLogin.type()) {
            case "html", "Delivery" -> "redirect:/";
            case "string" -> "";
            case "int" -> 400;
            default -> null;
        };
    }
}

