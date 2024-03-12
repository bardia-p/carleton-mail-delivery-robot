package com.cmds.webapp.models;

import com.fasterxml.jackson.annotation.JsonBackReference;
import com.fasterxml.jackson.annotation.JsonManagedReference;
import com.fasterxml.jackson.databind.annotation.JsonAppend;
import jakarta.persistence.*;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

import java.util.ArrayList;
import java.util.List;


/**
 * Class Robot for defining a Robot entity.
 */
@Entity
@Getter
@Setter
@NoArgsConstructor
public class Robot {
    @Id
    private String name;


    @OneToMany(mappedBy = "assignedRobot", fetch = FetchType.EAGER, cascade = CascadeType.ALL, orphanRemoval = true)
    @JsonManagedReference
    private List<Delivery> listTrips;

    private boolean shouldDie;

    /**
     * Enum RobotStatus for defining Robot status entities.
     */

    public enum RobotStatus {
        IDLE, BUSY;
    }
    private RobotStatus status;
    /**
     * One constructor for a Robot. Mostly used for testing purposes.
     * @param name String name.
     * @param status RobotStatus Enum status.
     * @param listTrips List of Delivery object trips.
     */
    public Robot(String name, RobotStatus status, List<Delivery> listTrips){
        this.name = name;
        this.status = status;
        this.listTrips = listTrips;
        this.shouldDie = false;
    }

    /**
     * Primary constructor for a Robot. This is the one that should be used in most cases.
     * @param name String name.
     */
    public Robot(String name) {
        this.name = name;
        this.status = RobotStatus.IDLE;
        this.listTrips = new ArrayList<>();
        this.shouldDie = false;
    }

    /**
     * Void method for adding a trip.
     * @param delivery Delivery object delivery.
     */
    public void addTrip (Delivery delivery){
        this.setStatus(RobotStatus.BUSY);
        this.listTrips.add(delivery);
    }

    /**
     * Boolean method for removing a trip from the list of trips.
     * @param tripId A Long trip ID.
     * @return True if the trip was removed, false otherwise.
     */
    public boolean removeTrip (Long tripId) {
        return this.listTrips.removeIf(trip -> trip.getDeliveryId().equals(tripId));
    }
}
