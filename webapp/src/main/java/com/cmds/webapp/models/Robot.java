package com.cmds.webapp.models;

import jakarta.persistence.Entity;
import jakarta.persistence.Id;
import jakarta.persistence.OneToMany;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

import java.util.List;

@Entity
@Getter
@Setter
@NoArgsConstructor
public class Robot {
    @Id
    public Long robotId;

    public String status;

    @OneToMany
    public List<Delivery> listTrips;

    public Robot(Long robotId, String status, List<Delivery> listTrips){
        this.robotId = robotId;
        this.status = status;
        this.listTrips = listTrips;
    }
}
