package com.cmds.webapp;

import com.cmds.webapp.models.Superuser;
import com.cmds.webapp.repos.SuperuserRepository;
import org.springframework.boot.CommandLineRunner;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.context.annotation.Bean;

/**
 * Class that boots up the Spring Boot application.
 */
@SpringBootApplication
public class WebappApplication {

    public static void main(String[] args) {
        SpringApplication.run(WebappApplication.class, args);
    }

    /**
     * This bean initializes an admin account for the team's use.
     * @param superuserRepo The superuser repository.
     * @return
     */
    @Bean
    public CommandLineRunner bean(SuperuserRepository superuserRepo) {
        return (args) -> {
            Superuser superuser = new Superuser("admin", "sysc4907");
            superuserRepo.save(superuser);
        };
    }
}
