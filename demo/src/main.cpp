#include <SFML/Graphics.hpp>
#include "../../src/dot_engine/engine.hpp"
#include "../../src/dot_engine/components/body/limited_dynamic_rigid_body.hpp"
#include "../../src/dot_engine/components/body/static_rigid_body.hpp"
#include "../../src/dot_engine/components/universal_law/gravity.hpp"
#include "../../src/dot_engine/components/universal_law/drag.hpp"
#include "../../src/dot_engine/components/force/targeted_force.hpp"
#include "../../src/dot_engine/components/force/link.hpp"
#include "../../src/dot_engine/components/force/jump.hpp"
#include "../../src/dot_engine/components/collision_effect/blocking.hpp"
#include <chrono>
#include <iostream>
#include <atomic>
#include <thread>
#include <mutex>
#include <random>

DotEngine engine;
bool end = false;
std::mutex physic_lock;

float delta_t_sum;
float delta_t_phys_sum;
int delta_t_nbr;

void physic_loop()
{

    const std::chrono::microseconds physic_delta_t_micro = std::chrono::microseconds(10000);
    std::chrono::microseconds physic_time = std::chrono::microseconds(0);
    float physic_delta_t = float(physic_delta_t_micro.count())/1000000.0;
    std::cout << "physic_dt = " << physic_delta_t << std::endl;

    const std::chrono::duration physic_time_0 = std::chrono::system_clock::now().time_since_epoch();

    while(!end)
    {
        const std::chrono::duration start_time = std::chrono::system_clock::now().time_since_epoch();

        // Update de la physique
        physic_lock.lock();
        engine.update(physic_delta_t, 10);
        physic_lock.unlock();
        const std::chrono::duration post_physic_time = std::chrono::system_clock::now().time_since_epoch();
        
        // Sleep if we are too fast
        physic_time += physic_delta_t_micro;
        if( physic_time.count() > std::chrono::duration_cast<std::chrono::microseconds>(post_physic_time-physic_time_0).count())
        {
            std::this_thread::sleep_for(physic_delta_t_micro);
        }
        std::chrono::duration end_time = std::chrono::system_clock::now().time_since_epoch();

        // Monitor dt
        std::chrono::microseconds delta_t_micro = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        const std::chrono::microseconds physic_compute_time = std::chrono::duration_cast<std::chrono::microseconds>(post_physic_time - start_time);

        const float delta_t = float(delta_t_micro.count());
        const float delta_t_phys = float(physic_compute_time.count());

        physic_lock.lock();
        delta_t_nbr += 1;
        delta_t_sum+= delta_t;
        delta_t_phys_sum += delta_t_phys;
        physic_lock.unlock();

    }
}

int main()
{
    auto window = sf::RenderWindow(sf::VideoMode({1080, 720}), "CMake SFML Project");
    window.setFramerateLimit(60);

    // Ajout des loi universelle
    std::shared_ptr<DotUniversalLawDrag> drag_law = std::make_shared<DotUniversalLawDrag>(DotUniversalLawDrag(0.5));
    engine.register_universal_law(drag_law);

    std::shared_ptr<DotUniversalLawAstralGravity> gravity_law = std::make_shared<DotUniversalLawAstralGravity>(DotUniversalLawAstralGravity(1000.0));
    engine.register_universal_law(gravity_law);

    // Ajout des effets de collision
    std::shared_ptr<DotBlockingCollisionEffect> blocking_collision_effect = std::make_shared<DotBlockingCollisionEffect>(DotBlockingCollisionEffect());
    engine.register_collision_effect(blocking_collision_effect);


    const float hard_hardness = 50000.0;
    const float obj_damping = 500.0;
    const float max_speed = 500.0;
    const float default_player_size = 30.0;
    std::shared_ptr<DotLimitedDynamicRigidBody> player_ptr = std::make_shared<DotLimitedDynamicRigidBody>(DotLimitedDynamicRigidBody());
    player_ptr->set_hardness(hard_hardness);
    player_ptr->set_damping(obj_damping);
    player_ptr->set_mass(20.0);
    player_ptr->set_size(default_player_size);
    player_ptr->set_position(Float2d(540.0, -300.0));
    player_ptr->set_max_speed(max_speed);
    engine.register_body(player_ptr);

    std::mt19937 gen( 0 );
    std::uniform_real_distribution<float> dist( -50000, 50000 );
    const size_t nbr_useless_particules = 10000;
    for(size_t i = 0 ; i < nbr_useless_particules; i++)
    {
        std::shared_ptr<DotLimitedDynamicRigidBody> truc_ptr = std::make_shared<DotLimitedDynamicRigidBody>(DotLimitedDynamicRigidBody());
        truc_ptr->set_hardness(0.0);
        truc_ptr->set_damping(0.0);
        truc_ptr->set_mass(1.0);
        truc_ptr->set_size(1.0);
        truc_ptr->set_position(Float2d(dist(gen), -dist(gen)));
        truc_ptr->set_max_speed(max_speed);
        truc_ptr->set_weak_collision(true);
        engine.register_body(std::move(truc_ptr));
    }

    std::shared_ptr<DotLimitedDynamicRigidBody> ball_ptr_1 = std::make_shared<DotLimitedDynamicRigidBody>(DotLimitedDynamicRigidBody());
    ball_ptr_1->set_hardness(hard_hardness);
    ball_ptr_1->set_damping(obj_damping);
    ball_ptr_1->set_mass(5.0);
    ball_ptr_1->set_size(10.0);
    ball_ptr_1->set_position(Float2d(540.0, -200.0));
    ball_ptr_1->set_max_speed(max_speed);
    engine.register_body(ball_ptr_1);

    std::shared_ptr<DotLimitedDynamicRigidBody> ball_ptr_2 = std::make_shared<DotLimitedDynamicRigidBody>(DotLimitedDynamicRigidBody());
    ball_ptr_2->set_hardness(hard_hardness);
    ball_ptr_2->set_damping(obj_damping);
    ball_ptr_2->set_mass(5.0);
    ball_ptr_2->set_size(10.0);
    ball_ptr_2->set_position(Float2d(510.0, -200.0));
    ball_ptr_2->set_max_speed(max_speed);
    engine.register_body(ball_ptr_2);

    std::shared_ptr<DotLimitedDynamicRigidBody> ball_ptr_3 = std::make_shared<DotLimitedDynamicRigidBody>(DotLimitedDynamicRigidBody());
    ball_ptr_3->set_hardness(hard_hardness);
    ball_ptr_3->set_damping(obj_damping);
    ball_ptr_3->set_mass(5.0);
    ball_ptr_3->set_size(10.0);
    ball_ptr_3->set_position(Float2d(570.0, -200.0));
    ball_ptr_3->set_max_speed(max_speed);
    engine.register_body(ball_ptr_3);

    // Ajout d'un sol
    std::shared_ptr<DotStaticRigidBody> ground_ptr = std::make_shared<DotStaticRigidBody>(DotStaticRigidBody());
    ground_ptr->set_hardness(hard_hardness);
    ground_ptr->set_damping(obj_damping);
    ground_ptr->set_mass(1000000.0);
    ground_ptr->set_size(1000.0);
    ground_ptr->set_position(Float2d(480.0, -1500));
    ground_ptr->set_weak_collision(true);
    engine.register_body(ground_ptr);
    gravity_law->register_star(ground_ptr);

    // Ajout des forces
    const float player_ctrl_force = 10000;
    std::shared_ptr<DotTargetedForce> player_controler_force = std::make_shared<DotTargetedForce>(DotTargetedForce());
    player_controler_force->set_target(player_ptr);
    engine.register_force(player_controler_force);

    const float player_jmp_force = 100000;
    std::shared_ptr<DotJumpingForce> player_jump_force = std::make_shared<DotJumpingForce>(DotJumpingForce());
    player_jump_force->set_jumper(player_ptr);
    player_jump_force->add_wall(ground_ptr);
    player_jump_force->set_degradation_rate(player_jmp_force*4);
    player_jump_force->set_distance_threshold(4.0);
    player_jump_force->set_initial_value(player_jmp_force);
    engine.register_force(player_jump_force);

    const float ball_spring_hardness = 50000.0; //hard_hardness;
    const float ball_spring_damping = 500.0; //obj_damping;
    const float ball_spring_length = 30.0;
    std::shared_ptr<DotSpringLink> spring_1_2 = std::make_shared<DotSpringLink>(DotSpringLink());
    spring_1_2->set_hardness(ball_spring_hardness);
    spring_1_2->set_damping(ball_spring_damping);
    spring_1_2->set_length(ball_spring_length);
    spring_1_2->set_target_a(ball_ptr_1);
    spring_1_2->set_target_b(ball_ptr_2);
    engine.register_force(spring_1_2);

    std::shared_ptr<DotSpringLink> spring_1_3 = std::make_shared<DotSpringLink>(DotSpringLink());
    spring_1_3->set_hardness(ball_spring_hardness);
    spring_1_3->set_damping(ball_spring_damping);
    spring_1_3->set_length(ball_spring_length);
    spring_1_3->set_target_a(ball_ptr_1);
    spring_1_3->set_target_b(ball_ptr_3);
    engine.register_force(spring_1_3);

    std::shared_ptr<DotSpringLink> spring_2_3 = std::make_shared<DotSpringLink>(DotSpringLink());
    spring_2_3->set_hardness(ball_spring_hardness);
    spring_2_3->set_damping(ball_spring_damping);
    spring_2_3->set_length(ball_spring_length);
    spring_2_3->set_target_a(ball_ptr_2);
    spring_2_3->set_target_b(ball_ptr_3);
    engine.register_force(spring_2_3);


    // Ajout des formes pour l'affichage
    sf::CircleShape player_circle;
    player_circle.setFillColor(sf::Color(0, 255, 10));
    player_circle.setRadius(player_ptr->get_size());
    player_circle.setOrigin(sf::Vector2f(player_ptr->get_size(), player_ptr->get_size()));
    player_circle.setPosition(sf::Vector2f(player_ptr->get_position().x(), -player_ptr->get_position().y()));
    
    sf::CircleShape ball_circle_1;
    ball_circle_1.setFillColor(sf::Color(255, 0, 0));
    ball_circle_1.setRadius(ball_ptr_1->get_size());
    ball_circle_1.setOrigin(sf::Vector2f(ball_ptr_1->get_size(), ball_ptr_1->get_size()));
    ball_circle_1.setPosition(sf::Vector2f(ball_ptr_1->get_position().x(), -ball_ptr_1->get_position().y()));

    sf::CircleShape ball_circle_2;
    ball_circle_2.setFillColor(sf::Color(255, 0, 0));
    ball_circle_2.setRadius(ball_ptr_2->get_size());
    ball_circle_2.setOrigin(sf::Vector2f(ball_ptr_2->get_size(), ball_ptr_2->get_size()));
    ball_circle_2.setPosition(sf::Vector2f(ball_ptr_2->get_position().x(), -ball_ptr_2->get_position().y()));

    sf::CircleShape ball_circle_3;
    ball_circle_3.setFillColor(sf::Color(255, 0, 0));
    ball_circle_3.setRadius(ball_ptr_3->get_size());
    ball_circle_3.setOrigin(sf::Vector2f(ball_ptr_3->get_size(), ball_ptr_3->get_size()));
    ball_circle_3.setPosition(sf::Vector2f(ball_ptr_3->get_position().x(), -ball_ptr_3->get_position().y()));

    sf::ConvexShape ball_triangle;
    ball_triangle.setFillColor(sf::Color(255,0,0));
    ball_triangle.setPointCount(3);
    ball_triangle.setPoint(0, sf::Vector2f(ball_ptr_1->get_position().x(), -ball_ptr_1->get_position().y()));
    ball_triangle.setPoint(1, sf::Vector2f(ball_ptr_2->get_position().x(), -ball_ptr_2->get_position().y()));
    ball_triangle.setPoint(2, sf::Vector2f(ball_ptr_3->get_position().x(), -ball_ptr_3->get_position().y()));

    sf::CircleShape ground_circle;
    ground_circle.setFillColor(sf::Color(255, 255, 255));
    ground_circle.setRadius(ground_ptr->get_size());
    ground_circle.setOrigin(sf::Vector2f(ground_ptr->get_size(), ground_ptr->get_size()));
    ground_circle.setPosition(sf::Vector2f(ground_ptr->get_position().x(), -ground_ptr->get_position().y()));

    std::thread physic_thread( physic_loop );

    // values to get
    float player_size = player_ptr->get_size();
    Float2d player_position = player_ptr->get_position();
    Float2d ball_1_position = ball_ptr_1->get_position();
    Float2d ball_2_position = ball_ptr_2->get_position();
    Float2d ball_3_position = ball_ptr_3->get_position();
    Float2d ground_position = ground_ptr->get_position();

    // values to set
    Float2d set_player_ctrl_force = Float2d();
    float player_jump = false;

    //  Print counter
    int delta_print_itt = 0;


    // Boucle de simulation / affichage
    while (window.isOpen())
    {

        while (const std::optional event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
            {
                window.close();
            }
        }

        // Gesion des contrôles
        bool key_press = false;
        set_player_ctrl_force = Float2d();
        if( sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W)) {set_player_ctrl_force.y() += player_ctrl_force; key_press = true;}
        if( sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S)) {set_player_ctrl_force.y() -= player_ctrl_force; key_press = true;}
        if( sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A)) {set_player_ctrl_force.x() -= player_ctrl_force; key_press = true;}
        if( sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D)) {set_player_ctrl_force.x() += player_ctrl_force; key_press = true;}
        if(! key_press)
        {
            //Si aucune commande, on freine
            set_player_ctrl_force = -5.0 * player_ptr->get_speed();
        }
        if( (set_player_ctrl_force.norm2()*0.98) > (player_ctrl_force*player_ctrl_force) )
        {
            set_player_ctrl_force = set_player_ctrl_force.normalised() * player_ctrl_force;
        }

        if( sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Space)) {
            player_jump = true;
        }
        else
        {
            player_jump = false;
        }

        // Communication avec le moteur physique
        physic_lock.lock();
        // values to set
        player_jump_force->set_is_active(player_jump);
        player_controler_force->set_value(set_player_ctrl_force);

        // values to get
        player_size = player_ptr->get_size();
        player_position = player_ptr->get_position();
        ball_1_position = ball_ptr_1->get_position();
        ball_2_position = ball_ptr_2->get_position();
        ball_3_position = ball_ptr_3->get_position();
        ground_position = ground_ptr->get_position();

        // print dt
        if(delta_print_itt >= 60)
        {
            std::cout << "Mean Physic Loop Time \t\t= " << (delta_t_sum/float(delta_t_nbr)) << "us" << std::endl;
            std::cout << "Mean Physic Computation Time \t= " << (delta_t_phys_sum/float(delta_t_nbr)) << "us" << std::endl;
            delta_t_nbr = 0;
            delta_t_sum = 0.0;
            delta_t_phys_sum = 0.0;
            delta_print_itt = 0;
        }
        else
        {
            delta_print_itt += 1;
        }

        physic_lock.unlock();

        // Mise à jour des formes à afficher
        player_circle.setRadius(player_size);
        player_circle.setOrigin(sf::Vector2f(player_size, player_size));
        player_circle.setPosition(sf::Vector2f(player_position.x(), -player_position.y()));

        ball_circle_1.setPosition(sf::Vector2f(ball_1_position.x(), -ball_1_position.y()));
        ball_circle_2.setPosition(sf::Vector2f(ball_2_position.x(), -ball_2_position.y()));
        ball_circle_3.setPosition(sf::Vector2f(ball_3_position.x(), -ball_3_position.y()));
        ground_circle.setPosition(sf::Vector2f(ground_position.x(), -ground_position.y()));

        ball_triangle.setPoint(0, sf::Vector2f(ball_1_position.x(), -ball_1_position.y()));
        ball_triangle.setPoint(1, sf::Vector2f(ball_2_position.x(), -ball_2_position.y()));
        ball_triangle.setPoint(2, sf::Vector2f(ball_3_position.x(), -ball_3_position.y()));

        // Affichage
        window.clear();
        window.draw(ball_triangle);
        window.draw(ground_circle);
        window.draw(ball_circle_1);
        window.draw(ball_circle_2);
        window.draw(ball_circle_3);
        window.draw(player_circle);
        window.display();
    }

    physic_lock.lock();
    end = true;
    physic_lock.unlock();
    if( physic_thread.joinable()) physic_thread.join();
}