package com.cmds.webapp.repos;

import com.cmds.webapp.models.AppUser;
import org.springframework.data.repository.CrudRepository;

import java.util.List;
import java.util.Optional;

/**
 * The repository in charge of managing the CRUD operations for the User Entity.
 */
public interface UserRepository extends CrudRepository<AppUser, Long>{
    List<AppUser> findAll();

    Optional<AppUser> findByUsername(String userid);
}

