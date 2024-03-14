package com.cmds.webapp.repos;

import com.cmds.webapp.models.AppUser;
import com.cmds.webapp.models.Superuser;
import org.springframework.data.repository.CrudRepository;

import java.util.List;
import java.util.Optional;

/**
 * The repository in charge of managing the CRUD operations for the Superuser Entity.
 */
public interface SuperuserRepository extends CrudRepository<Superuser, Long>{
    List<Superuser> findAll();

    Optional<Superuser> findByUsername(String userid);
}

