package com.cmds.webapp.models;

import com.fasterxml.jackson.annotation.JsonBackReference;
import com.fasterxml.jackson.annotation.JsonManagedReference;
import jakarta.persistence.*;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

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

    public String startingDest;

    public String finalDest;

    public String status;


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
        this.status = "";
        this.assignedRobot = null;
    }
}
