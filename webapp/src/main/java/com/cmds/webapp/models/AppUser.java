package com.cmds.webapp.models;

import jakarta.persistence.*;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

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
    @GeneratedValue(strategy = GenerationType.AUTO)
    public Long userId;
    // Define a username for the user
    private String username;

    // Define a password for the associated username
    private String password;

    @OneToOne
    private Delivery currentDelivery;

    /**
     * Default constructor for User.
     * @param username A string username.
     * @param password A string password.
     */
    public AppUser(String username, String password) {
        this.username = username;
        this.password = password;
        this.currentDelivery = null;
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

