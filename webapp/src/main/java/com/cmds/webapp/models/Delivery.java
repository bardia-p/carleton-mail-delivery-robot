package com.cmds.webapp.models;

import jakarta.persistence.*;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

import java.util.ArrayList;
import java.util.List;

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

    /**
     * Default constructor for Delivery.
     * @param startingDest A string starting destination.
     * @param finalDest A string final destination.
     */
    public Delivery(String startingDest, String finalDest) {
        this.startingDest = startingDest;
        this.finalDest = finalDest;
        this.status = "";
    }
}
