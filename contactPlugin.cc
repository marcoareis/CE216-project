#include "contactPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

    /////////////////////////////////////////////////
ContactPlugin::ContactPlugin() : SensorPlugin()
{
    //x = 57.96;
    //test.Set(57.96);
}

/////////////////////////////////////////////////
ContactPlugin::~ContactPlugin()
{
}

/////////////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
    // Get the parent sensor.
    this->parentSensor =
        boost::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);
    // Make sure the parent sensor is valid.
    if (!this->parentSensor)
    {
        gzerr << "ContactPlugin requires a ContactSensor.\n";
        return;
    }

    // Connect to the sensor update event.
    this->updateConnection = this->parentSensor->ConnectUpdated(
            boost::bind(&ContactPlugin::OnUpdate, this));

    // Make sure the parent sensor is active.
    this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void ContactPlugin::OnUpdate()
{
    /*
    msgs::Contacts contacts;
    contacts = this->parentSensor->GetContacts();
    if (contacts.contact_size() && contacts.contact(0).position_size()) {
        this->pos.x = contacts.contact(0).position(0).x(); 
        this->pos.y = contacts.contact(0).position(0).y();
        this->pos.z = contacts.contact(0).position(0).z();
    }
    //*/

    msgs::Contacts contacts;
    contacts = this->parentSensor->GetContacts();
    for (unsigned int i = 0; i < contacts.contact_size(); ++i)
    {
        for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
        {
            /*
            this->x = (double) contacts.contact(i).position(j).x(); 
            this->y = (double) contacts.contact(i).position(j).y();
            this->z = (double) contacts.contact(i).position(j).z();
            this->pos.Set(x, y, z);
            gazebo::ModelPush::testFoo(x, y, z);
    std::cout << "plugin update:" << " x= " << pos.x << ", y= " << pos.y  << ", z=" << pos.z << std::endl;  
            std::cout << "xyz update: " << x << "," << y << "," << z << std::endl;
            //*/
            i = contacts.contact_size();
            break;
        }
    }

    /* Get all the contacts.
    msgs::Contacts contacts;
    contacts = this->parentSensor->GetContacts();
    for (unsigned int i = 0; i < contacts.contact_size(); ++i)
    {
        std::cout << i 
            << " Collision between[" << contacts.contact(i).collision1()
            << "] and [" << contacts.contact(i).collision2() << "]\n";

        for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
        {
            std::cout << j << "  Position:"
                << contacts.contact(i).position(j).x() << " "
                << contacts.contact(i).position(j).y() << " "
                << contacts.contact(i).position(j).z() << "\n";
            std::cout << "   Normal:"
                << contacts.contact(i).normal(j).x() << " "
                << contacts.contact(i).normal(j).y() << " "
                << contacts.contact(i).normal(j).z() << "\n";
            std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
        }
    }
    //*/
    return;
}

/*
void  ContactPlugin::getContactPos(math::Vector3 &out)
{
    out.x = this->pos.x; 
    out.y = this->pos.y;
    out.z = this->pos.z; 


    std::cout << "plugin:" << " x= " << pos.x << ", y= " << pos.y  << ", z=" << pos.z << std::endl;  
    std::cout << "test: " << test.x << "," << test.y << "," << test.z << std::endl;
    std::cout << "xyz: " << this->x << "," << this->y << "," << this->z << std::endl;
    //return out;
}
//*/
