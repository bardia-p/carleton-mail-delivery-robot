package com.cmds.webapp.models;

import jakarta.persistence.Entity;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

@Entity
@Getter
@Setter
@NoArgsConstructor
public class Superuser extends AppUser {
    public Superuser(String username, String password){
        super(username, password);
        this.type = UserType.SUPER_USER;
    }
}
