package com.cmds.webapp.repos;

import com.cmds.webapp.models.Robot;
import org.springframework.data.repository.CrudRepository;

import java.util.List;

/**
 * The repository in charge of managing the CRUD operations for the Robot Entity.
 */
public interface RobotRepository extends CrudRepository<Robot, Long> {
    List<Robot> findAll();
    Robot findByName(String Id);
}

