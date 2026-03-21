#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Pose3.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>



#include <iostream>

namespace gazebo
{
    class WclDebugPlugin : public ModelPlugin
    {
        public :
        //Here we are making first constructor of ModelPlugin and then wclDebugPlugin()
        WclDebugPlugin() : ModelPlugin()
        {


        }
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
        {
            this->model = _model;
            this->lastPrintTime = this->model->GetWorld()->SimTime();
            this->world = this->model->GetWorld();
            this->gzNode = gazebo::transport::NodePtr(new gazebo :: transport :: Node());
            this->gzNode->Init(this->world->Name()); // Connects to Gazebo’s internal message bus for that specific world.
            this->contactSub = this->gzNode->Subscribe("/gazebo/tank_world/physics/contacts",&WclDebugPlugin::OnContacts,this);


            auto contactManager = this->world->Physics()->GetContactManager();
contactManager->SetNeverDropContacts(false);

            std::cout <<"[wcl Debug Plugin] Loaded for model:"<<this->model->GetName()<< std::endl;
            //that below line  gives the every time the every time simulation update 
            // reads sensor data continewsly and  applies forces  and control joints  

            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&WclDebugPlugin::OnUpdate, this));

            //callback function 
            
        }
        
        void OnContacts(ConstContactsPtr &msg)
        {


            // Assume no tank contact in this message
           this->isTouchingTank = false;

          // Loop through all contacts in this message
           for (int i = 0; i < msg->contact_size(); ++i)
          {
             const auto &contact = msg->contact(i);

              std::string c1 =  contact.collision1();
                 std::string c2 =  contact.collision2();

                // Check if either side is the tank collision
                   if (c1.find("tank_collision") != std::string::npos ||
                              c2.find("tank_collision") != std::string::npos)
                                {
                                      this->isTouchingTank = true;
                                          break;  // one is enough
                                }

          }



        }

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
    
    double magneticStrength = 500.0;

    


     ApplyForceToWheel("wheel1_link",tankPos , magneticStrength );
    ApplyForceToWheel("wheel2_link",tankPos , magneticStrength );
    ApplyForceToWheel("wheel3_link",tankPos ,  magneticStrength);
    ApplyForceToWheel("wheel4_link",tankPos , magneticStrength);

     auto bodyLink = this->model->GetLink("body_link");
     if (bodyLink)
    {



        std::cout << "Applying magnetic force" << std::endl;

        ignition::math::Vector3d angVel = bodyLink->WorldAngularVel();

// Kill pitch rotation (Y axis)
    bodyLink->AddTorque(
    ignition::math::Vector3d(0.0, -80.0 * angVel.Y(), 0.0));


    //     double mass = bodyLink->GetInertial()->Mass();
    //   double gravityForce = mass * 9.81;

    //   ignition::math::Vector3d upward(0, 0, 0.75 * gravityForce);
    //   ignition::math::Vector3d totalForce = magneticForce + upward;
    //   bodyLink->AddForce(totalForce);

    }
    }

 }
    
    



     

            
}
        private:  

         void ApplyForceToWheel(const std::string &linkName,
                         const ignition::math::Vector3d &tankPos, double strength)
  {
    auto link = this->model->GetLink(linkName);
    if (link)
    {
      auto wheelPos = link->WorldPose().Pos();
  double dx = tankPos.X() - wheelPos.X();
  double dy = tankPos.Y() - wheelPos.Y();
  double mag = sqrt(dx*dx + dy*dy);
      link->AddForce(ignition :: math :: Vector3d( strength * dx / mag, strength * dy/mag , 0.0));
    }
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