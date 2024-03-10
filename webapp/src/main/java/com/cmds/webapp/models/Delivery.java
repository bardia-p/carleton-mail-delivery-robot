package com.cmds.webapp.models;

import com.fasterxml.jackson.annotation.JsonBackReference;
import jakarta.persistence.*;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

import java.util.ArrayList;
import java.util.List;

/**
 * Class Delivery for defining a Delivery entity.
 */
@Entity
@Getter
@Setter
@NoArgsConstructor
public class Delivery {
    @Id
    @GeneratedValue(strategy = GenerationType.AUTO)
    private Long deliveryId;

    private String startingDest;

    private String finalDest;

    @ElementCollection
    private List<String> statuses;


    @ManyToOne(fetch=FetchType.EAGER, cascade = CascadeType.ALL)
    @JsonBackReference
    private Robot assignedRobot;

    /**
     * Default constructor for Delivery.
     * @param startingDest A string starting destination.
     * @param finalDest A string final destination.
     */
    public Delivery(String startingDest, String finalDest) {
        this.startingDest = startingDest;
        this.finalDest = finalDest;
        this.statuses = new ArrayList<>(){{ add("NEW");}};
        this.assignedRobot = null;
    }

    /**
     * Adds a new status to the list of statuses.
     *
     * @param s the new status to add.
     */
    public void addStatus(String s){
        this.statuses.add(s);
    }
}
