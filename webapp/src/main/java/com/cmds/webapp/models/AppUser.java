package com.cmds.webapp.models;

import jakarta.persistence.*;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;
import org.apache.catalina.User;

import java.util.Objects;

/**
 * Class User for defining a User entity that uses the website.
 */
@Entity
@Getter
@Setter
@NoArgsConstructor
public class AppUser {
    @Id
    private String username;

    // Define a password for the associated username
    private String password;

    @OneToOne
    private Delivery currentDelivery;

    /**
     * Enum RobotStatus for defining Robot status entities.
     */

    @Getter
    public enum UserType {
        REGULAR("Regular"), SUPER_USER("Superuser");

        private final String type;
        UserType(String type) {
            this.type = type;
        }
    }

    public UserType type;

    /**
     * Default constructor for User.
     * @param username A string username.
     * @param password A string password.
     */
    public AppUser(String username, String password) {
        this.username = username;
        this.password = password;
        this.currentDelivery = null;
        this.type = UserType.REGULAR;
    }

    /**
     *
     * @param o object that is being compared with
     * @return boolean value saying whether objects are equal
     */
    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        AppUser appUser = (AppUser) o;
        return Objects.equals(username, appUser.username) && Objects.equals(password, appUser.password);
    }

}

