#include <SFML/Graphics.hpp>
#include "../../src/dot_engine/physic_thread.hpp"
#include "../../src/dot_engine/components/body/limited_dynamic_rigid_body.hpp"
#include "../../src/dot_engine/components/body/static_rigid_body.hpp"
#include "../../src/dot_engine/components/universal_law/gravity.hpp"
#include "../../src/dot_engine/components/universal_law/drag.hpp"
#include "../../src/dot_engine/components/force/targeted_force.hpp"
#include "../../src/dot_engine/components/force/link.hpp"
#include "../../src/dot_engine/components/force/jump.hpp"
#include "../../src/dot_engine/components/force/run.hpp"
#include "../../src/dot_engine/components/collision_effect/blocking.hpp"
#include <iostream>
#include <random>

int main()
{
    const float dt_second = 0.001;
    const uint8_t force_resolution_multiplier = 10;
    MonitoredPysicThread physic_thread(dt_second, force_resolution_multiplier);
    DotEngine& engine = physic_thread.engine();

    auto window = sf::RenderWindow(sf::VideoMode({1080, 720}), "CMake SFML Project");
    window.setFramerateLimit(60);

    // Ajout des loi universelle
    std::shared_ptr<DotUniversalLawDrag> drag_law = std::make_shared<DotUniversalLawDrag>(DotUniversalLawDrag(0.5));
    engine.register_system_low_resolution(drag_law);

    std::shared_ptr<DotUniversalLawAstralGravity> gravity_law = std::make_shared<DotUniversalLawAstralGravity>(DotUniversalLawAstralGravity(1000.0));
    engine.register_system_low_resolution(gravity_law);

    // Ajout des effets de collision
    std::shared_ptr<DotBlockingCollisionEffect> blocking_collision_effect = std::make_shared<DotBlockingCollisionEffect>(DotBlockingCollisionEffect());
    engine.register_system_high_resolution(blocking_collision_effect);


    const float hard_hardness = 50000.0;
    const float obj_damping = 500.0;
    const float max_speed = 500.0;
    const float default_player_size = 30.0;
    std::shared_ptr<DotLimitedDynamicRigidBody> player_ptr = std::make_shared<DotLimitedDynamicRigidBody>(DotLimitedDynamicRigidBody());
    player_ptr->set_hardness(hard_hardness);
    player_ptr->set_damping(obj_damping);
    player_ptr->set_mass(20.0);
    player_ptr->set_size(default_player_size);
    player_ptr->set_position(Float2d(540.0, -280.0));
    player_ptr->set_max_speed(max_speed);
    engine.register_body(player_ptr);

    std::mt19937 gen( 0 );
    std::uniform_real_distribution<float> dist( -50000, 50000 );
    const size_t nbr_useless_particules = 1000;
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
    ball_ptr_1->set_position(Float2d(540.0, -350.0));
    ball_ptr_1->set_max_speed(max_speed);
    engine.register_body(ball_ptr_1);

    std::shared_ptr<DotLimitedDynamicRigidBody> ball_ptr_2 = std::make_shared<DotLimitedDynamicRigidBody>(DotLimitedDynamicRigidBody());
    ball_ptr_2->set_hardness(hard_hardness);
    ball_ptr_2->set_damping(obj_damping);
    ball_ptr_2->set_mass(5.0);
    ball_ptr_2->set_size(10.0);
    ball_ptr_2->set_position(Float2d(510.0, -350.0));
    ball_ptr_2->set_max_speed(max_speed);
    engine.register_body(ball_ptr_2);

    std::shared_ptr<DotLimitedDynamicRigidBody> ball_ptr_3 = std::make_shared<DotLimitedDynamicRigidBody>(DotLimitedDynamicRigidBody());
    ball_ptr_3->set_hardness(hard_hardness);
    ball_ptr_3->set_damping(obj_damping);
    ball_ptr_3->set_mass(5.0);
    ball_ptr_3->set_size(10.0);
    ball_ptr_3->set_position(Float2d(570.0, -350.0));
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

    std::shared_ptr<DotStaticRigidBody> ground2_ptr = std::make_shared<DotStaticRigidBody>(DotStaticRigidBody());
    ground2_ptr->set_hardness(hard_hardness);
    ground2_ptr->set_damping(obj_damping);
    ground2_ptr->set_mass(1000000.0);
    ground2_ptr->set_size(1000.0);
    ground2_ptr->set_position(Float2d(480.0, 1000));
    ground2_ptr->set_weak_collision(true);
    engine.register_body(ground2_ptr);
    gravity_law->register_star(ground2_ptr);

    // Ajout des forces
    const float player_jmp_force = 100000;
    std::shared_ptr<DotJumpingForce> player_jump_force = std::make_shared<DotJumpingForce>(DotJumpingForce());
    player_jump_force->set_jumper(player_ptr);
    player_jump_force->add_wall(ground_ptr);
    player_jump_force->add_wall(ground2_ptr);
    player_jump_force->add_wall(ball_ptr_1);
    player_jump_force->add_wall(ball_ptr_2);
    player_jump_force->add_wall(ball_ptr_3);
    player_jump_force->set_degradation_rate(player_jmp_force*4);
    player_jump_force->set_distance_threshold(4.0);
    player_jump_force->set_initial_value(player_jmp_force);
    engine.register_system_low_resolution(player_jump_force);

    const float player_run_force_magnitude = 20000;
    std::shared_ptr<DotRunningForce> player_run_force = std::make_shared<DotRunningForce>(DotRunningForce());
    player_run_force->set_runner(player_ptr);
    player_run_force->add_floor(ball_ptr_1, 3.0);
    player_run_force->add_floor(ball_ptr_2, 3.0);
    player_run_force->add_floor(ball_ptr_3, 3.0);
    player_run_force->add_floor(ground_ptr, 1.0);
    player_run_force->add_floor(ground2_ptr, 1.0);
    player_run_force->set_distance_threshold(4.0);
    player_run_force->set_running_value(player_run_force_magnitude);
    engine.register_system_low_resolution(player_run_force);

    const float ball_spring_hardness = 10000.0; //hard_hardness;
    const float ball_spring_damping = 100.0; //obj_damping;
    const float ball_spring_length = 30.0;
    std::shared_ptr<DotSpringLink> spring_1_2 = std::make_shared<DotSpringLink>(DotSpringLink());
    spring_1_2->set_hardness(ball_spring_hardness);
    spring_1_2->set_damping(ball_spring_damping);
    spring_1_2->set_length(ball_spring_length);
    spring_1_2->set_target_a(ball_ptr_1);
    spring_1_2->set_target_b(ball_ptr_2);
    engine.register_system_high_resolution(spring_1_2);

    std::shared_ptr<DotSpringLink> spring_1_3 = std::make_shared<DotSpringLink>(DotSpringLink());
    spring_1_3->set_hardness(ball_spring_hardness);
    spring_1_3->set_damping(ball_spring_damping);
    spring_1_3->set_length(ball_spring_length);
    spring_1_3->set_target_a(ball_ptr_1);
    spring_1_3->set_target_b(ball_ptr_3);
    engine.register_system_high_resolution(spring_1_3);

    std::shared_ptr<DotSpringLink> spring_2_3 = std::make_shared<DotSpringLink>(DotSpringLink());
    spring_2_3->set_hardness(ball_spring_hardness);
    spring_2_3->set_damping(ball_spring_damping);
    spring_2_3->set_length(ball_spring_length);
    spring_2_3->set_target_a(ball_ptr_2);
    spring_2_3->set_target_b(ball_ptr_3);
    engine.register_system_high_resolution(spring_2_3);


    // Ajout des formes pour l'affichage
    float display_scaling = 10.0;
    sf::CircleShape player_circle;
    player_circle.setFillColor(sf::Color(0, 255, 10));
    player_circle.setRadius(player_ptr->get_size()*display_scaling);
    player_circle.setOrigin(sf::Vector2f(player_ptr->get_size()*display_scaling, player_ptr->get_size()*display_scaling));
    player_circle.setPosition(sf::Vector2f(player_ptr->get_position().x()*display_scaling, player_ptr->get_position().y()*display_scaling));
    
    sf::CircleShape ball_circle_1;
    ball_circle_1.setFillColor(sf::Color(255, 0, 0));
    ball_circle_1.setRadius(ball_ptr_1->get_size()*display_scaling);
    ball_circle_1.setOrigin(sf::Vector2f(ball_ptr_1->get_size()*display_scaling, ball_ptr_1->get_size()*display_scaling));
    ball_circle_1.setPosition(sf::Vector2f(ball_ptr_1->get_position().x()*display_scaling, ball_ptr_1->get_position().y()*display_scaling));

    sf::CircleShape ball_circle_2;
    ball_circle_2.setFillColor(sf::Color(255, 0, 0));
    ball_circle_2.setRadius(ball_ptr_2->get_size()*display_scaling);
    ball_circle_2.setOrigin(sf::Vector2f(ball_ptr_2->get_size()*display_scaling, ball_ptr_2->get_size()*display_scaling));
    ball_circle_2.setPosition(sf::Vector2f(ball_ptr_2->get_position().x()*display_scaling, ball_ptr_2->get_position().y()*display_scaling));

    sf::CircleShape ball_circle_3;
    ball_circle_3.setFillColor(sf::Color(255, 0, 0));
    ball_circle_3.setRadius(ball_ptr_3->get_size()*display_scaling);
    ball_circle_3.setOrigin(sf::Vector2f(ball_ptr_3->get_size()*display_scaling, ball_ptr_3->get_size()*display_scaling));
    ball_circle_3.setPosition(sf::Vector2f(ball_ptr_3->get_position().x()*display_scaling, ball_ptr_3->get_position().y()*display_scaling));

    sf::ConvexShape ball_triangle;
    ball_triangle.setFillColor(sf::Color(255,0,0));
    ball_triangle.setPointCount(3);
    ball_triangle.setPoint(0, sf::Vector2f(ball_ptr_1->get_position().x()*display_scaling, ball_ptr_1->get_position().y()*display_scaling));
    ball_triangle.setPoint(1, sf::Vector2f(ball_ptr_2->get_position().x()*display_scaling, ball_ptr_2->get_position().y()*display_scaling));
    ball_triangle.setPoint(2, sf::Vector2f(ball_ptr_3->get_position().x()*display_scaling, ball_ptr_3->get_position().y()*display_scaling));

    sf::CircleShape ground_circle;
    ground_circle.setPointCount(64);
    ground_circle.setFillColor(sf::Color(255, 255, 255));
    ground_circle.setRadius(ground_ptr->get_size()*display_scaling);
    ground_circle.setOrigin(sf::Vector2f(ground_ptr->get_size()*display_scaling, ground_ptr->get_size()*display_scaling));
    ground_circle.setPosition(sf::Vector2f(ground_ptr->get_position().x()*display_scaling, ground_ptr->get_position().y()*display_scaling));

    sf::CircleShape ground2_circle;
    ground2_circle.setPointCount(64);
    ground2_circle.setFillColor(sf::Color(255, 255, 255));
    ground2_circle.setRadius(ground2_ptr->get_size()*display_scaling);
    ground2_circle.setOrigin(sf::Vector2f(ground2_ptr->get_size()*display_scaling, ground2_ptr->get_size()*display_scaling));
    ground2_circle.setPosition(sf::Vector2f(ground2_ptr->get_position().x()*display_scaling, ground2_ptr->get_position().y()*display_scaling));

    physic_thread.start();

    // values to get
    float player_size = player_ptr->get_size();
    Float2d player_position = player_ptr->get_position();
    Float2d ball_1_position = ball_ptr_1->get_position();
    Float2d ball_2_position = ball_ptr_2->get_position();
    Float2d ball_3_position = ball_ptr_3->get_position();
    Float2d ground_1_position = ground_ptr->get_position();
    Float2d ground_2_position = ground2_ptr->get_position();

    // values to set
    int8_t set_player_run_dir = 0;
    float player_jump = false;

    //  Print counter
    int delta_print_itt = 0;

    // camera
    sf::View view(sf::FloatRect({0.f*display_scaling, 0.f*display_scaling}, {1080*display_scaling, 720*display_scaling}));
    float view_angle_deg = 180.0;
    view.setRotation(sf::degrees(view_angle_deg));

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
        set_player_run_dir = 0;
        if( sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A)) {set_player_run_dir += 1;}
        if( sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D)) {set_player_run_dir -= 1;}

        if( sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Space)) {
            player_jump = true;
        }
        else
        {
            player_jump = false;
        }

        // Communication avec le moteur physique
        physic_thread.lock();
        // values to set
        player_jump_force->set_is_active(player_jump);
        player_run_force->set_direction(set_player_run_dir);

        // values to get
        player_size = player_ptr->get_size();
        player_position = player_ptr->get_position();
        ball_1_position = ball_ptr_1->get_position();
        ball_2_position = ball_ptr_2->get_position();
        ball_3_position = ball_ptr_3->get_position();
        ground_1_position = ground_ptr->get_position();
        ground_2_position = ground2_ptr->get_position();

        // print dt
        if(delta_print_itt >= 60)
        {
            std::cout << "Mean Physic Loop Time \t\t= " << physic_thread.get_loop_time_second_mean()*1000.0 << "ms" << std::endl;
            std::cout << "Mean Physic Computation Time \t= " << physic_thread.get_physic_compute_time_second_mean()*1000.0 << "ms" << std::endl;
            delta_print_itt = 0;
        }
        else
        {
            delta_print_itt += 1;
        }

        physic_thread.unlock();

        // Mise à jour des formes à afficher
        player_circle.setRadius(player_size*display_scaling);
        player_circle.setOrigin(sf::Vector2f(player_size*display_scaling, player_size*display_scaling));
        player_circle.setPosition(sf::Vector2f(player_position.x()*display_scaling, player_position.y()*display_scaling));

        ball_circle_1.setPosition(sf::Vector2f(ball_1_position.x()*display_scaling, ball_1_position.y()*display_scaling));
        ball_circle_2.setPosition(sf::Vector2f(ball_2_position.x()*display_scaling, ball_2_position.y()*display_scaling));
        ball_circle_3.setPosition(sf::Vector2f(ball_3_position.x()*display_scaling, ball_3_position.y()*display_scaling));

        ball_triangle.setPoint(0, sf::Vector2f(ball_1_position.x()*display_scaling, ball_1_position.y()*display_scaling));
        ball_triangle.setPoint(1, sf::Vector2f(ball_2_position.x()*display_scaling, ball_2_position.y()*display_scaling));
        ball_triangle.setPoint(2, sf::Vector2f(ball_3_position.x()*display_scaling, ball_3_position.y()*display_scaling));

        // Mouvement de la vue
        float view_filter_alpha = 0.97;
        const float view_filter_beta = 1.0 - view_filter_alpha;

        // position
        const sf::Vector2f player_pos = player_circle.getPosition();
        const sf::Vector2f view_pos = view.getCenter();
        const sf::Vector2 new_view_pos = sf::Vector2f( (view_filter_beta*player_pos.x) + (view_filter_alpha*view_pos.x), (view_filter_beta*player_pos.y) + (view_filter_alpha*view_pos.y)  );
        view.setCenter( new_view_pos );

        // orientation
        const Float2d grav_diff_1 = player_position - ground_1_position;
        const Float2d grav_diff_2 = player_position - ground_2_position;
        const Float2d grav_dir_1 = grav_diff_1.normalised();
        const Float2d grav_dir_2 = grav_diff_2.normalised();
        const Float2d grav_diff_total = (grav_dir_1/grav_diff_1.norm2()) + (grav_dir_2/grav_diff_2.norm2());
        
        float player_angle = -((-grav_diff_total.angle_deg()) - 90.0);
        while(player_angle < 0.0) player_angle += 360.0;

        float diff_angle = player_angle - view_angle_deg;
        while(diff_angle < -180.0) diff_angle += 360.0;
        while(diff_angle > 180.0) diff_angle -= 360.0;

        const float rotation_beta = 0.01;
        view_angle_deg += rotation_beta * diff_angle;
        while(view_angle_deg < 0.0) view_angle_deg += 360.0;

        view.setRotation(sf::degrees(view_angle_deg));

        // Affichage
        window.clear();
        window.setView(view);
        window.draw(ball_triangle);
        window.draw(ground_circle);
        window.draw(ground2_circle);
        window.draw(ball_circle_1);
        window.draw(ball_circle_2);
        window.draw(ball_circle_3);
        window.draw(player_circle);
        window.display();
    }

    physic_thread.stop();
}