void OnUpdate()
        {
            auto pose = this->model->WorldPose();
            auto contactManager = this->world->Physics()->GetContactManager();
            auto contacts = contactManager->GetContacts();
        //     std::cout << "[WCL Debug Plugin] Contacts this frame: "
        //   << contacts.size() << std::endl;


            // for( auto &contact : contacts)
            // {
            //     std :: string c1 = contact->collision1->GetName();
            //     std :: string c2 = contact->collision2->GetName();
            //     std :: cout <<"[WCL Debug Plugin] Contact: "<<c1<<"<->"<<c2<<std::endl;
            // }


            auto currentTime = this->model->GetWorld()->SimTime();
            // if((currentTime - lastPrintTime) < 1.0  )
            // {
            //     return;
            // }
           
            
            // std :: cout<<"[wcl Debug Plugin ] Robot Pose:"
            // <<"x="<<pose.Pos().X()
            // <<"y="<<pose.Pos().Y()
            // <<", z=" <<pose.Pos().Z()<<std :: endl;
            // lastPrintTime = currentTime;


            if (this->isTouchingTank && !this->wasTouchingTank)
{
//   std::cout << "[WCL Debug Plugin] Touching tank" << std::endl;
}

this->wasTouchingTank = this->isTouchingTank;

if (this->isTouchingTank)
{
    //std::cout << "Applying magnetic force" << std::endl;


    auto robotPos = this->model->WorldPose().Pos();
    auto tankModel = this->world->ModelByName("cylindrical_tank");
    if (tankModel)
    {

    auto tankPos = tankModel->WorldPose().Pos();
    double dx = tankPos.X() - robotPos.X();
    double dy = tankPos.Y() - robotPos.Y();
    // double dz = tankPos.Z() - robotPos.Z();

    double magnitude = sqrt(dx*dx + dy*dy );
    dx = dx/magnitude;
    dy = dy/magnitude;
    // dz = dz/magnitude;

    ignition::math::Vector3d dir(dx, dy, 0.0);
    double magneticStrength = 100.0;

    double fx = magneticStrength * dir.X();
    double fy = magneticStrength * dir.Y();
    double fz = magneticStrength * dir.Z();

    ignition::math::Vector3d magneticForce(fx, fy, fz);

    


     ApplyForceToWheel("wheel1_link",magneticForce );
    ApplyForceToWheel("wheel2_link",magneticForce );
    ApplyForceToWheel("wheel3_link", magneticForce);
    ApplyForceToWheel("wheel4_link",magneticForce );

     auto bodyLink = this->model->GetLink("body_link");
     if (bodyLink)
    {
        double mass = bodyLink->GetInertial()->Mass();
      double gravityForce = mass * 9.81;

      ignition::math::Vector3d upward(0, 0, 0.75 * gravityForce);
      bodyLink->AddForce(upward);


    }
    }

 }
    
    



     

            
}
        private:  

         void ApplyForceToWheel(const std::string &linkName,
                         const ignition::math::Vector3d &magneticForce)
  {
    auto link = this->model->GetLink(linkName);
    if (link)
      link->AddForce(magneticForce);
  }
        physics ::ModelPtr model;
        event::ConnectionPtr updateConnection;
        common::Time lastPrintTime;
        physics :: WorldPtr world;
        gazebo :: transport :: NodePtr gzNode;
        gazebo :: transport::SubscriberPtr contactSub;
        bool isTouchingTank = false;
        bool wasTouchingTank = false;





    };
    GZ_REGISTER_MODEL_PLUGIN(WclDebugPlugin)
}