package com.cmds.webapp.repos;

import com.cmds.webapp.models.Delivery;
import org.springframework.data.repository.CrudRepository;

import java.util.List;
import java.util.Optional;

/**
 * The repository in charge of managing the CRUD operations for the User Entity.
 */
public interface DeliveryRepository extends CrudRepository<Delivery, Long> {
    List<Delivery> findAll();
    Delivery findById(long Id);
}

