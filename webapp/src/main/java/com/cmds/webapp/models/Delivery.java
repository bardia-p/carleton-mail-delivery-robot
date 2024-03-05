package com.cmds.webapp.models;

import jakarta.persistence.Entity;
import jakarta.persistence.GeneratedValue;
import jakarta.persistence.GenerationType;
import jakarta.persistence.Id;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

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

    /**
     * Default constructor for Delivery.
     * @param startingDest A string starting destination.
     * @param finalDest A string final destination.
     */
    public Delivery(String startingDest, String finalDest) {
        this.startingDest = startingDest;
        this.finalDest = finalDest;
    }
}
