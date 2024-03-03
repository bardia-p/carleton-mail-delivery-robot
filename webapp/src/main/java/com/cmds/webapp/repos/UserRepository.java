package com.cmds.webapp.repos;

import com.cmds.webapp.models.User;
import org.springframework.data.repository.CrudRepository;

import java.util.List;
import java.util.Optional;

/**
 * The repository in charge of managing the CRUD operations for the User Entity.
 */
public interface UserRepository extends CrudRepository<User, Long>{
    List<User> findAll();

    Optional<User> findByUsername(String userid);
}

