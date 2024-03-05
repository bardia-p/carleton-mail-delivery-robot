package com.cmds.webapp.repos;

import com.cmds.webapp.models.Robot;
import org.springframework.data.repository.CrudRepository;

import java.util.List;
import java.util.Optional;

/**
 * The repository in charge of managing the CRUD operations for the User Entity.
 */
public interface RobotRepository extends CrudRepository<Robot, Long> {
    List<Robot> findAll();
    Robot findById(long Id);
}

