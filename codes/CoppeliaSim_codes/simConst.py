#constants
#Scene object types. Values are serialized
sim_object_shape_type           =0
sim_object_joint_type           =1
sim_object_graph_type           =2
sim_object_camera_type          =3
sim_object_dummy_type           =4
sim_object_proximitysensor_type =5
sim_object_reserved1            =6
sim_object_reserved2            =7
sim_object_path_type            =8
sim_object_visionsensor_type    =9
sim_object_volume_type          =10
sim_object_mill_type            =11
sim_object_forcesensor_type     =12
sim_object_light_type           =13
sim_object_mirror_type          =14

#General object types. Values are serialized
sim_appobj_object_type          =109
sim_appobj_collision_type       =110
sim_appobj_distance_type        =111
sim_appobj_simulation_type      =112
sim_appobj_ik_type              =113
sim_appobj_constraintsolver_type=114
sim_appobj_collection_type      =115
sim_appobj_ui_type              =116
sim_appobj_script_type          =117
sim_appobj_pathplanning_type    =118
sim_appobj_RESERVED_type        =119
sim_appobj_texture_type         =120

# Ik calculation methods. Values are serialized
sim_ik_pseudo_inverse_method        =0
sim_ik_damped_least_squares_method  =1
sim_ik_jacobian_transpose_method    =2

# Ik constraints. Values are serialized
sim_ik_x_constraint         =1
sim_ik_y_constraint         =2
sim_ik_z_constraint         =4
sim_ik_alpha_beta_constraint=8
sim_ik_gamma_constraint     =16
sim_ik_avoidance_constraint =64

# Ik calculation results 
sim_ikresult_not_performed  =0
sim_ikresult_success        =1
sim_ikresult_fail           =2

# Scene object sub-types. Values are serialized 
# Light sub-types 
sim_light_omnidirectional_subtype   =1
sim_light_spot_subtype              =2
sim_light_directional_subtype       =3
# Joint sub-types 
sim_joint_revolute_subtype          =10
sim_joint_prismatic_subtype         =11
sim_joint_spherical_subtype         =12
# Shape sub-types 
sim_shape_simpleshape_subtype       =20
sim_shape_multishape_subtype        =21
# Proximity sensor sub-types 
sim_proximitysensor_pyramid_subtype =30
sim_proximitysensor_cylinder_subtype=31
sim_proximitysensor_disc_subtype    =32
sim_proximitysensor_cone_subtype    =33
sim_proximitysensor_ray_subtype     =34
# Mill sub-types 
sim_mill_pyramid_subtype            =40
sim_mill_cylinder_subtype           =41
sim_mill_disc_subtype               =42
sim_mill_cone_subtype               =42
# No sub-type 
sim_object_no_subtype               =200


#Scene object main properties (serialized)
sim_objectspecialproperty_collidable                    =0x0001
sim_objectspecialproperty_measurable                    =0x0002
#reserved                        =0x0004 
#reserved                        =0x0008 
sim_objectspecialproperty_detectable_ultrasonic            =0x0010
sim_objectspecialproperty_detectable_infrared            =0x0020
sim_objectspecialproperty_detectable_laser                =0x0040
sim_objectspecialproperty_detectable_inductive            =0x0080
sim_objectspecialproperty_detectable_capacitive            =0x0100
sim_objectspecialproperty_renderable                    =0x0200
sim_objectspecialproperty_detectable_all =sim_objectspecialproperty_detectable_ultrasonic|sim_objectspecialproperty_detectable_infrared|sim_objectspecialproperty_detectable_laser|sim_objectspecialproperty_detectable_inductive|sim_objectspecialproperty_detectable_capacitive
sim_objectspecialproperty_cuttable                        =0x0400
sim_objectspecialproperty_pathplanning_ignored            =0x0800

# Model properties (serialized)
sim_modelproperty_not_collidable                =0x0001
sim_modelproperty_not_measurable                =0x0002
sim_modelproperty_not_renderable                =0x0004
sim_modelproperty_not_detectable                =0x0008
sim_modelproperty_not_cuttable                    =0x0010
sim_modelproperty_not_dynamic                    =0x0020
sim_modelproperty_not_respondable                =0x0040 # cannot be selected if sim_modelproperty_not_dynamic is not selected 
sim_modelproperty_not_reset                        =0x0080 # Model is not reset at simulation end. This flag is cleared at simulation end 
sim_modelproperty_not_visible                    =0x0100 # Whole model is invisible independent of local visibility settings 
sim_modelproperty_not_model                        =0xf000 # object is not a model 


# Check the documentation instead of comments below!! 
# Following messages are dispatched to the Lua-message container 
sim_message_ui_button_state_change  =0    # a UI button slider etc. changed (due to a user's action). aux[0]=UI handle aux[1]=button handle aux[2]=button attributes aux[3]=slider position (if slider) 
sim_message_reserved9               =1    # Do not use 
sim_message_object_selection_changed=2
sim_message_reserved10                =3    # do not use 
sim_message_model_loaded            =4
sim_message_reserved11                =5    # do not use 
sim_message_keypress                =6    # a key was pressed while the focus was on a page (aux[0]=key aux[1]=ctrl and shift key state) 
sim_message_bannerclicked            =7    # a banner was clicked (aux[0]=banner ID) 


# Following messages are dispatched only to the C-API (not available from Lua) 
sim_message_for_c_api_only_start        =0x100      # Do not use 
sim_message_reserved1                   =0x101      # Do not use 
sim_message_reserved2                    =0x102      # Do not use 
sim_message_reserved3                    =0x103      # Do not use 
sim_message_eventcallback_scenesave        =0x104        # about to save a scene 
sim_message_eventcallback_modelsave        =0x105      # about to save a model (current selection will be saved) 
sim_message_eventcallback_moduleopen    =0x106        # called when simOpenModule in Lua is called 
sim_message_eventcallback_modulehandle    =0x107        # called when simHandleModule in Lua is called with argument false 
sim_message_eventcallback_moduleclose    =0x108        # called when simCloseModule in Lua is called 
sim_message_reserved4                    =0x109      # Do not use 
sim_message_reserved5                    =0x10a        # Do not use 
sim_message_reserved6                    =0x10b        # Do not use 
sim_message_reserved7                    =0x10c        # Do not use 
sim_message_eventcallback_instancepass    =0x10d        # Called once every main application loop pass. auxiliaryData[0] contains event flags of events that happened since last time 
sim_message_eventcallback_broadcast     =0x10e
sim_message_eventcallback_imagefilter_enumreset =0x10f
sim_message_eventcallback_imagefilter_enumerate      =0x110
sim_message_eventcallback_imagefilter_adjustparams   =0x111
sim_message_eventcallback_imagefilter_reserved       =0x112
sim_message_eventcallback_imagefilter_process        =0x113
sim_message_eventcallback_reserved1                  =0x114   # do not use 
sim_message_eventcallback_reserved2                  =0x115   # do not use 
sim_message_eventcallback_reserved3                  =0x116   # do not use 
sim_message_eventcallback_reserved4                  =0x117   # do not use 
sim_message_eventcallback_abouttoundo                 =0x118   # the undo button was hit and a previous state is about to be restored 
sim_message_eventcallback_undoperformed                 =0x119   # the undo button was hit and a previous state restored 
sim_message_eventcallback_abouttoredo                 =0x11a   # the redo button was hit and a future state is about to be restored  
sim_message_eventcallback_redoperformed                 =0x11b   # the redo button was hit and a future state restored  
sim_message_eventcallback_scripticondblclick         =0x11c   # scipt icon was double clicked.  (aux[0]=object handle associated with script set replyData[0] to 1 if script should not be opened)  
sim_message_eventcallback_simulationabouttostart     =0x11d
sim_message_eventcallback_simulationended            =0x11e
sim_message_eventcallback_reserved5                     =0x11f   # do not use 
sim_message_eventcallback_keypress                     =0x120   # a key was pressed while the focus was on a page (aux[0]=key aux[1]=ctrl and shift key state) 
sim_message_eventcallback_modulehandleinsensingpart  =0x121   # called when simHandleModule in Lua is called with argument true 
sim_message_eventcallback_renderingpass              =0x122   # called just before the scene is rendered 
sim_message_eventcallback_bannerclicked              =0x123   # called when a banner was clicked (aux[0]=banner ID) 
sim_message_eventcallback_menuitemselected           =0x124   # auxiliaryData[0] indicates the handle of the item auxiliaryData[1] indicates the state of the item 
sim_message_eventcallback_refreshdialogs             =0x125   # aux[0]=refresh degree (0=light 1=medium 2=full) 
sim_message_eventcallback_sceneloaded                =0x126
sim_message_eventcallback_modelloaded                =0x127
sim_message_eventcallback_instanceswitch             =0x128
sim_message_eventcallback_guipass                    =0x129
sim_message_eventcallback_mainscriptabouttobecalled  =0x12a
sim_message_eventcallback_rmlposition                =0x12b   #the command simRMLPosition was called. The appropriate plugin should handle the call
sim_message_eventcallback_rmlvelocity                =0x12c   # the command simRMLVelocity was called. The appropriate plugin should handle the call
sim_message_simulation_start_resume_request          =0x1000
sim_message_simulation_pause_request                 =0x1001
sim_message_simulation_stop_request                  =0x1002

# Scene object properties. Combine with the | operator 
sim_objectproperty_reserved1                =0x0000
sim_objectproperty_reserved2                =0x0001
sim_objectproperty_reserved3                =0x0002
sim_objectproperty_reserved4                =0x0003
sim_objectproperty_reserved5                =0x0004 # formely sim_objectproperty_visible 
sim_objectproperty_reserved6                =0x0008 # formely sim_objectproperty_wireframe 
sim_objectproperty_collapsed                =0x0010
sim_objectproperty_selectable                =0x0020
sim_objectproperty_reserved7                =0x0040
sim_objectproperty_selectmodelbaseinstead    =0x0080
sim_objectproperty_dontshowasinsidemodel    =0x0100
# reserved                                    =0x0200 
sim_objectproperty_canupdatedna                =0x0400
sim_objectproperty_selectinvisible            =0x0800
sim_objectproperty_depthinvisible            =0x1000


# type of arguments (input and output) for custom lua commands 
sim_lua_arg_nil     =0
sim_lua_arg_bool    =1    
sim_lua_arg_int     =2
sim_lua_arg_float   =3
sim_lua_arg_string  =4
sim_lua_arg_invalid =5
sim_lua_arg_table   =8

# custom user interface properties. Values are serialized. 
sim_ui_property_visible                        =0x0001
sim_ui_property_visibleduringsimulationonly    =0x0002
sim_ui_property_moveable                    =0x0004
sim_ui_property_relativetoleftborder        =0x0008
sim_ui_property_relativetotopborder            =0x0010
sim_ui_property_fixedwidthfont                =0x0020
sim_ui_property_systemblock                    =0x0040
sim_ui_property_settocenter                    =0x0080
sim_ui_property_rolledup                    =0x0100
sim_ui_property_selectassociatedobject        =0x0200
sim_ui_property_visiblewhenobjectselected    =0x0400


# button properties. Values are serialized. 
sim_buttonproperty_button                =0x0000
sim_buttonproperty_label                =0x0001
sim_buttonproperty_slider                =0x0002
sim_buttonproperty_editbox                =0x0003
sim_buttonproperty_staydown                =0x0008
sim_buttonproperty_enabled                =0x0010
sim_buttonproperty_borderless            =0x0020
sim_buttonproperty_horizontallycentered    =0x0040
sim_buttonproperty_ignoremouse            =0x0080
sim_buttonproperty_isdown                =0x0100
sim_buttonproperty_transparent            =0x0200
sim_buttonproperty_nobackgroundcolor    =0x0400
sim_buttonproperty_rollupaction            =0x0800
sim_buttonproperty_closeaction            =0x1000
sim_buttonproperty_verticallycentered    =0x2000
sim_buttonproperty_downupevent            =0x4000


# Simulation status 
sim_simulation_stopped                        =0x00                                # Simulation is stopped 
sim_simulation_paused                        =0x08                                # Simulation is paused 
sim_simulation_advancing                    =0x10                                # Simulation is advancing 
sim_simulation_advancing_firstafterstop        =sim_simulation_advancing|0x00        # First simulation pass (1x) 
sim_simulation_advancing_running            =sim_simulation_advancing|0x01        # Normal simulation pass (>=1x) 
# reserved                                    =sim_simulation_advancing|0x02 
sim_simulation_advancing_lastbeforepause    =sim_simulation_advancing|0x03        # Last simulation pass before pause (1x) 
sim_simulation_advancing_firstafterpause    =sim_simulation_advancing|0x04        # First simulation pass after pause (1x) 
sim_simulation_advancing_abouttostop        =sim_simulation_advancing|0x05        # "Trying to stop" simulation pass (>=1x) 
sim_simulation_advancing_lastbeforestop        =sim_simulation_advancing|0x06        # Last simulation pass (1x) 


# Script execution result (first return value) 
sim_script_no_error                    =0
sim_script_main_script_nonexistent    =1
sim_script_main_script_not_called    =2
sim_script_reentrance_error            =4
sim_script_lua_error                =8
sim_script_call_error                =16


 # Script types (serialized!) 
sim_scripttype_mainscript   =0
sim_scripttype_childscript  =1
sim_scripttype_jointctrlcallback  =4
sim_scripttype_contactcallback  =5
sim_scripttype_customizationscript  =6
sim_scripttype_generalcallback  =7

# API call error messages 
sim_api_errormessage_ignore    =0    # does not memorize nor output errors 
sim_api_errormessage_report    =1    # memorizes errors (default for C-API calls) 
sim_api_errormessage_output    =2  # memorizes and outputs errors (default for Lua-API calls) 


# special argument of some functions 
sim_handle_all                        =-2
sim_handle_all_except_explicit        =-3
sim_handle_self                        =-4
sim_handle_main_script                =-5
sim_handle_tree                        =-6
sim_handle_chain                    =-7
sim_handle_single                    =-8
sim_handle_default                    =-9
sim_handle_all_except_self            =-10
sim_handle_parent                    =-11


# special handle flags
sim_handleflag_assembly                =0x400000
sim_handleflag_model                =0x800000


# distance calculation methods (serialized) 
sim_distcalcmethod_dl               =0
sim_distcalcmethod_dac              =1
sim_distcalcmethod_max_dl_dac       =2
sim_distcalcmethod_dl_and_dac       =3
sim_distcalcmethod_sqrt_dl2_and_dac2=4
sim_distcalcmethod_dl_if_nonzero    =5
sim_distcalcmethod_dac_if_nonzero   =6


 # Generic dialog styles 
sim_dlgstyle_message        =0
sim_dlgstyle_input          =1
sim_dlgstyle_ok             =2
sim_dlgstyle_ok_cancel      =3
sim_dlgstyle_yes_no         =4
sim_dlgstyle_dont_center    =32# can be combined with one of above values. Only with this flag can the position of the related UI be set just after dialog creation  

 # Generic dialog return values 
sim_dlgret_still_open   =0
sim_dlgret_ok           =1
sim_dlgret_cancel       =2
sim_dlgret_yes          =3
sim_dlgret_no           =4


# Path properties 
sim_pathproperty_show_line                            =0x0001
sim_pathproperty_show_orientation                    =0x0002
sim_pathproperty_closed_path                        =0x0004
sim_pathproperty_automatic_orientation                =0x0008
sim_pathproperty_invert_velocity                    =0x0010
sim_pathproperty_infinite_acceleration                =0x0020
sim_pathproperty_flat_path                            =0x0040
sim_pathproperty_show_position                        =0x0080
sim_pathproperty_auto_velocity_profile_translation    =0x0100
sim_pathproperty_auto_velocity_profile_rotation        =0x0200
sim_pathproperty_endpoints_at_zero                    =0x0400
sim_pathproperty_keep_x_up                            =0x0800


 # drawing objects 
# following are mutually exclusive 
sim_drawing_points          =0            # 3 values per point (point size in pixels) 
sim_drawing_lines            =1            # 6 values per line (line size in pixels) 
sim_drawing_triangles        =2            # 9 values per triangle 
sim_drawing_trianglepoints    =3            # 6 values per point (3 for triangle position 3 for triangle normal vector) (triangle size in meters) 
sim_drawing_quadpoints        =4            # 6 values per point (3 for quad position 3 for quad normal vector) (quad size in meters) 
sim_drawing_discpoints        =5            # 6 values per point (3 for disc position 3 for disc normal vector) (disc size in meters) 
sim_drawing_cubepoints        =6          # 6 values per point (3 for cube position 3 for cube normal vector) (cube size in meters) 
sim_drawing_spherepoints    =7          # 3 values per point (sphere size in meters) 

# following can be or-combined 
sim_drawing_itemcolors                =0x00020 # +3 values per item (each item has its own ambient color (rgb values)).
                                             # Mutually exclusive with sim_drawing_vertexcolors 
sim_drawing_vertexcolors            =0x00040 # +3 values per vertex (each vertex has its own ambient color (rgb values). Only for sim_drawing_lines (+6) and for sim_drawing_triangles(+9)). Mutually exclusive with sim_drawing_itemcolors 
sim_drawing_itemsizes                =0x00080 # +1 value per item (each item has its own size). Not for sim_drawing_triangles 
sim_drawing_backfaceculling            =0x00100 # back faces are not displayed for all items 
sim_drawing_wireframe                =0x00200 # all items displayed in wireframe 
sim_drawing_painttag                =0x00400 # all items are tagged as paint (for additinal processing at a later stage) 
sim_drawing_followparentvisibility    =0x00800 # if the object is associated with a scene object then it follows that visibility otherwise it is always visible 
sim_drawing_cyclic                    =0x01000 # if the max item count was reached then the first items are overwritten. 
sim_drawing_50percenttransparency    =0x02000 # the drawing object will be 50% transparent 
sim_drawing_25percenttransparency    =0x04000 # the drawing object will be 25% transparent 
sim_drawing_12percenttransparency    =0x08000 # the drawing object will be 12.5% transparent 
sim_drawing_emissioncolor            =0x10000 # When used in combination with sim_drawing_itemcolors or sim_drawing_vertexcolors then the specified colors will be for the emissive component 
sim_drawing_facingcamera            =0x20000 # Only for trianglepoints quadpoints discpoints and cubepoints. If specified the normal verctor is calculated to face the camera (each item data requires 3 values less) 
sim_drawing_overlay                    =0x40000 # When specified objects are always drawn on top of "regular objects" 
sim_drawing_itemtransparency        =0x80000  # +1 value per item (each item has its own transparency value (0-1)). Not compatible with sim_drawing_vertexcolors 

# banner values 
# following can be or-combined 
sim_banner_left                        =0x00001 # Banners display on the left of the specified point 
sim_banner_right                    =0x00002 # Banners display on the right of the specified point 
sim_banner_nobackground                =0x00004 # Banners have no background rectangle 
sim_banner_overlay                    =0x00008 # When specified banners are always drawn on top of "regular objects" 
sim_banner_followparentvisibility    =0x00010 # if the object is associated with a scene object then it follows that visibility otherwise it is always visible 
sim_banner_clickselectsparent        =0x00020 # if the object is associated with a scene object then clicking the banner will select the scene object 
sim_banner_clicktriggersevent        =0x00040 # if the banner is clicked an event is triggered (sim_message_eventcallback_bannerclicked and sim_message_bannerclicked are generated) 
sim_banner_facingcamera                =0x00080 # If specified the banner will always face the camera by rotating around the banner's vertical axis (y-axis) 
sim_banner_fullyfacingcamera        =0x00100 # If specified the banner will always fully face the camera (the banner's orientation is same as the camera looking at it) 
sim_banner_backfaceculling            =0x00200 # If specified the banner will only be visible from one side 
sim_banner_keepsamesize                =0x00400 # If specified the banner will always appear in the same size. In that case size represents the character height in pixels 
sim_banner_bitmapfont                =0x00800 # If specified a fixed-size bitmap font is used. The text will also always fully face the camera and be right 
                                             # to the specified position. Bitmap fonts are not clickable 


# particle objects following are mutually exclusive 
sim_particle_points1        =0  # 6 values per point (pt1 and pt2. Pt1 is start position pt2-pt1 is the initial velocity vector). i
                                #Point is 1 pixel big. Only appearance is a point internally handled as a perfect sphere 
sim_particle_points2        =1    # 6 values per point. Point is 2 pixel big. Only appearance is a point internally handled as a perfect sphere 
sim_particle_points4        =2    # 6 values per point. Point is 4 pixel big. Only appearance is a point internally handled as a perfect sphere 
sim_particle_roughspheres    =3    # 6 values per sphere. Only appearance is rough. Internally a perfect sphere 
sim_particle_spheres        =4    # 6 values per sphere. Internally a perfect sphere 




# following can be or-combined 
sim_particle_respondable1to4        =0x0020 # the particles are respondable against shapes (against all objects that have at least one bit 1-4 activated in the global respondable mask) 
sim_particle_respondable5to8        =0x0040 # the particles are respondable against shapes (against all objects that have at least one bit 5-8 activated in the global respondable mask) 
sim_particle_particlerespondable    =0x0080 # the particles are respondable against each other 
sim_particle_ignoresgravity            =0x0100 # the particles ignore the effect of gravity. Not compatible with sim_particle_water 
sim_particle_invisible                =0x0200 # the particles are invisible 
sim_particle_itemsizes                =0x0400 # +1 value per particle (each particle can have a different size) 
sim_particle_itemdensities            =0x0800 # +1 value per particle (each particle can have a different density) 
sim_particle_itemcolors                =0x1000 # +3 values per particle (each particle can have a different color) 
sim_particle_cyclic                    =0x2000 # if the max item count was reached then the first items are overwritten. 
sim_particle_emissioncolor            =0x4000 # When used in combination with sim_particle_itemcolors then the specified colors will be for the emissive component 
sim_particle_water                    =0x8000 # the particles are water particles (no weight in the water (i.e. when z<0)). Not compatible with sim_particle_ignoresgravity 
sim_particle_painttag                =0x10000 # The particles can be seen by vision sensors (sim_particle_invisible must not be set) 




# custom user interface menu attributes 
sim_ui_menu_title        =1
sim_ui_menu_minimize    =2
sim_ui_menu_close        =4
sim_ui_menu_systemblock =8



# Boolean parameters 
sim_boolparam_hierarchy_visible                 =0
sim_boolparam_console_visible                   =1
sim_boolparam_collision_handling_enabled        =2
sim_boolparam_distance_handling_enabled         =3
sim_boolparam_ik_handling_enabled               =4
sim_boolparam_gcs_handling_enabled              =5
sim_boolparam_dynamics_handling_enabled         =6
sim_boolparam_joint_motion_handling_enabled     =7
sim_boolparam_path_motion_handling_enabled      =8
sim_boolparam_proximity_sensor_handling_enabled =9
sim_boolparam_vision_sensor_handling_enabled    =10
sim_boolparam_mill_handling_enabled             =11
sim_boolparam_browser_visible                   =12
sim_boolparam_scene_and_model_load_messages     =13
sim_reserved0                                   =14
sim_boolparam_shape_textures_are_visible        =15
sim_boolparam_display_enabled                   =16
sim_boolparam_infotext_visible                  =17
sim_boolparam_statustext_open                   =18
sim_boolparam_fog_enabled                       =19
sim_boolparam_rml2_available                    =20
sim_boolparam_rml4_available                    =21
sim_boolparam_mirrors_enabled                    =22
sim_boolparam_aux_clip_planes_enabled            =23
sim_boolparam_full_model_copy_from_api            =24
sim_boolparam_realtime_simulation                =25
sim_boolparam_force_show_wireless_emission        =27
sim_boolparam_force_show_wireless_reception        =28
sim_boolparam_video_recording_triggered            =29
sim_boolparam_threaded_rendering_enabled        =32
sim_boolparam_fullscreen                        =33
sim_boolparam_headless                            =34
sim_boolparam_hierarchy_toolbarbutton_enabled    =35
sim_boolparam_browser_toolbarbutton_enabled        =36
sim_boolparam_objectshift_toolbarbutton_enabled    =37
sim_boolparam_objectrotate_toolbarbutton_enabled=38
sim_boolparam_force_calcstruct_all_visible        =39
sim_boolparam_force_calcstruct_all                =40
sim_boolparam_exit_request                        =41
sim_boolparam_play_toolbarbutton_enabled        =42
sim_boolparam_pause_toolbarbutton_enabled        =43
sim_boolparam_stop_toolbarbutton_enabled        =44
sim_boolparam_waiting_for_trigger                =45


# Integer parameters 
sim_intparam_error_report_mode      =0  # Check sim_api_errormessage_... constants above for valid values 
sim_intparam_program_version        =1  # e.g Version 2.1.4 --> 20104. Can only be read 
sim_intparam_instance_count         =2  # do not use anymore (always returns 1 since CoppeliaSim 2.5.11) 
sim_intparam_custom_cmd_start_id    =3  # can only be read 
sim_intparam_compilation_version    =4  # 0=evaluation version 1=full version 2=player version. Can only be read 
sim_intparam_current_page           =5
sim_intparam_flymode_camera_handle  =6  # can only be read 
sim_intparam_dynamic_step_divider   =7  # can only be read 
sim_intparam_dynamic_engine         =8  # 0=Bullet 1=ODE. 2=Vortex.
sim_intparam_server_port_start      =9  # can only be read 
sim_intparam_server_port_range      =10 # can only be read 
sim_intparam_visible_layers         =11
sim_intparam_infotext_style         =12
sim_intparam_settings               =13
sim_intparam_edit_mode_type         =14 # can only be read 
sim_intparam_server_port_next       =15 # is initialized at sim_intparam_server_port_start 
sim_intparam_qt_version             =16 # version of the used Qt framework 
sim_intparam_event_flags_read       =17 # can only be read 
sim_intparam_event_flags_read_clear =18 # can only be read 
sim_intparam_platform               =19 # can only be read 
sim_intparam_scene_unique_id        =20 # can only be read 
sim_intparam_work_thread_count      =21
sim_intparam_mouse_x                =22
sim_intparam_mouse_y                =23
sim_intparam_core_count             =24
sim_intparam_work_thread_calc_time_ms =25
sim_intparam_idle_fps               =26
sim_intparam_prox_sensor_select_down =27
sim_intparam_prox_sensor_select_up  =28
sim_intparam_stop_request_counter   =29
sim_intparam_program_revision       =30
sim_intparam_mouse_buttons          =31
sim_intparam_dynamic_warning_disabled_mask =32
sim_intparam_simulation_warning_disabled_mask =33
sim_intparam_scene_index            =34
sim_intparam_motionplanning_seed    =35
sim_intparam_speedmodifier          =36

# Float parameters 
sim_floatparam_rand=0 # random value (0.0-1.0) 
sim_floatparam_simulation_time_step =1
sim_floatparam_stereo_distance        =2

# String parameters 
sim_stringparam_application_path=0 # path of CoppeliaSim's executable 
sim_stringparam_video_filename=1
sim_stringparam_app_arg1            =2
sim_stringparam_app_arg2            =3
sim_stringparam_app_arg3            =4
sim_stringparam_app_arg4            =5
sim_stringparam_app_arg5            =6
sim_stringparam_app_arg6            =7
sim_stringparam_app_arg7            =8
sim_stringparam_app_arg8            =9
sim_stringparam_app_arg9            =10
sim_stringparam_scene_path_and_name    =13

# Array parameters 
sim_arrayparam_gravity          =0
sim_arrayparam_fog              =1
sim_arrayparam_fog_color        =2
sim_arrayparam_background_color1=3
sim_arrayparam_background_color2=4
sim_arrayparam_ambient_light    =5
sim_arrayparam_random_euler        =6

sim_objintparam_visibility_layer= 10
sim_objfloatparam_abs_x_velocity= 11
sim_objfloatparam_abs_y_velocity= 12
sim_objfloatparam_abs_z_velocity= 13
sim_objfloatparam_abs_rot_velocity= 14
sim_objfloatparam_objbbox_min_x= 15
sim_objfloatparam_objbbox_min_y= 16
sim_objfloatparam_objbbox_min_z= 17
sim_objfloatparam_objbbox_max_x= 18
sim_objfloatparam_objbbox_max_y= 19
sim_objfloatparam_objbbox_max_z= 20
sim_objfloatparam_modelbbox_min_x= 21
sim_objfloatparam_modelbbox_min_y= 22
sim_objfloatparam_modelbbox_min_z= 23
sim_objfloatparam_modelbbox_max_x= 24
sim_objfloatparam_modelbbox_max_y= 25
sim_objfloatparam_modelbbox_max_z= 26
sim_objintparam_collection_self_collision_indicator= 27
sim_objfloatparam_transparency_offset= 28
sim_objintparam_child_role= 29
sim_objintparam_parent_role= 30
sim_objintparam_manipulation_permissions= 31
sim_objintparam_illumination_handle= 32

sim_visionfloatparam_near_clipping= 1000
sim_visionfloatparam_far_clipping= 1001
sim_visionintparam_resolution_x= 1002
sim_visionintparam_resolution_y= 1003
sim_visionfloatparam_perspective_angle= 1004
sim_visionfloatparam_ortho_size= 1005
sim_visionintparam_disabled_light_components= 1006
sim_visionintparam_rendering_attributes= 1007
sim_visionintparam_entity_to_render= 1008
sim_visionintparam_windowed_size_x= 1009
sim_visionintparam_windowed_size_y= 1010
sim_visionintparam_windowed_pos_x= 1011
sim_visionintparam_windowed_pos_y= 1012
sim_visionintparam_pov_focal_blur= 1013
sim_visionfloatparam_pov_blur_distance= 1014
sim_visionfloatparam_pov_aperture= 1015
sim_visionintparam_pov_blur_sampled= 1016
sim_visionintparam_render_mode= 1017

sim_jointintparam_motor_enabled= 2000
sim_jointintparam_ctrl_enabled= 2001
sim_jointfloatparam_pid_p= 2002
sim_jointfloatparam_pid_i= 2003
sim_jointfloatparam_pid_d= 2004
sim_jointfloatparam_intrinsic_x= 2005
sim_jointfloatparam_intrinsic_y= 2006
sim_jointfloatparam_intrinsic_z= 2007
sim_jointfloatparam_intrinsic_qx= 2008
sim_jointfloatparam_intrinsic_qy= 2009
sim_jointfloatparam_intrinsic_qz= 2010
sim_jointfloatparam_intrinsic_qw= 2011
sim_jointfloatparam_velocity= 2012
sim_jointfloatparam_spherical_qx= 2013
sim_jointfloatparam_spherical_qy= 2014
sim_jointfloatparam_spherical_qz= 2015
sim_jointfloatparam_spherical_qw= 2016
sim_jointfloatparam_upper_limit= 2017
sim_jointfloatparam_kc_k= 2018
sim_jointfloatparam_kc_c= 2019
sim_jointfloatparam_ik_weight= 2021
sim_jointfloatparam_error_x= 2022
sim_jointfloatparam_error_y= 2023
sim_jointfloatparam_error_z= 2024
sim_jointfloatparam_error_a= 2025
sim_jointfloatparam_error_b= 2026
sim_jointfloatparam_error_g= 2027
sim_jointfloatparam_error_pos= 2028
sim_jointfloatparam_error_angle= 2029
sim_jointintparam_velocity_lock= 2030
sim_jointintparam_vortex_dep_handle= 2031
sim_jointfloatparam_vortex_dep_multiplication= 2032
sim_jointfloatparam_vortex_dep_offset= 2033

sim_shapefloatparam_init_velocity_x= 3000
sim_shapefloatparam_init_velocity_y= 3001
sim_shapefloatparam_init_velocity_z= 3002
sim_shapeintparam_static= 3003
sim_shapeintparam_respondable= 3004
sim_shapefloatparam_mass= 3005
sim_shapefloatparam_texture_x= 3006
sim_shapefloatparam_texture_y= 3007
sim_shapefloatparam_texture_z= 3008
sim_shapefloatparam_texture_a= 3009
sim_shapefloatparam_texture_b= 3010
sim_shapefloatparam_texture_g= 3011
sim_shapefloatparam_texture_scaling_x= 3012
sim_shapefloatparam_texture_scaling_y= 3013
sim_shapeintparam_culling= 3014
sim_shapeintparam_wireframe= 3015
sim_shapeintparam_compound= 3016
sim_shapeintparam_convex= 3017
sim_shapeintparam_convex_check= 3018
sim_shapeintparam_respondable_mask= 3019
sim_shapefloatparam_init_velocity_a= 3020
sim_shapefloatparam_init_velocity_b= 3021
sim_shapefloatparam_init_velocity_g= 3022
sim_shapestringparam_color_name= 3023
sim_shapeintparam_edge_visibility= 3024
sim_shapefloatparam_shading_angle= 3025
sim_shapefloatparam_edge_angle= 3026
sim_shapeintparam_edge_borders_hidden= 3027

sim_proxintparam_ray_invisibility= 4000

sim_forcefloatparam_error_x= 5000
sim_forcefloatparam_error_y= 5001
sim_forcefloatparam_error_z= 5002
sim_forcefloatparam_error_a= 5003
sim_forcefloatparam_error_b= 5004
sim_forcefloatparam_error_g= 5005
sim_forcefloatparam_error_pos= 5006
sim_forcefloatparam_error_angle= 5007

sim_lightintparam_pov_casts_shadows= 8000

sim_cameraintparam_disabled_light_components= 9000
sim_camerafloatparam_perspective_angle= 9001
sim_camerafloatparam_ortho_size= 9002
sim_cameraintparam_rendering_attributes= 9003
sim_cameraintparam_pov_focal_blur= 9004
sim_camerafloatparam_pov_blur_distance= 9005
sim_camerafloatparam_pov_aperture= 9006
sim_cameraintparam_pov_blur_samples= 9007

sim_dummyintparam_link_type= 10000

sim_mirrorfloatparam_width= 12000
sim_mirrorfloatparam_height= 12001
sim_mirrorfloatparam_reflectance= 12002
sim_mirrorintparam_enable= 12003

sim_pplanfloatparam_x_min= 20000
sim_pplanfloatparam_x_range= 20001
sim_pplanfloatparam_y_min= 20002
sim_pplanfloatparam_y_range= 20003
sim_pplanfloatparam_z_min= 20004
sim_pplanfloatparam_z_range= 20005
sim_pplanfloatparam_delta_min= 20006
sim_pplanfloatparam_delta_range= 20007

sim_mplanintparam_nodes_computed= 25000
sim_mplanintparam_prepare_nodes= 25001
sim_mplanintparam_clear_nodes= 25002

# User interface elements 
sim_gui_menubar                        =0x0001
sim_gui_popups                        =0x0002
sim_gui_toolbar1                    =0x0004
sim_gui_toolbar2                    =0x0008
sim_gui_hierarchy                    =0x0010
sim_gui_infobar                        =0x0020
sim_gui_statusbar                    =0x0040
sim_gui_scripteditor                =0x0080
sim_gui_scriptsimulationparameters    =0x0100
sim_gui_dialogs                        =0x0200
sim_gui_browser                        =0x0400
sim_gui_all                            =0xffff


# Joint modes 
sim_jointmode_passive       =0
sim_jointmode_motion        =1
sim_jointmode_ik            =2
sim_jointmode_ikdependent   =3
sim_jointmode_dependent     =4
sim_jointmode_force         =5


# Navigation and selection modes with the mouse. Lower byte values are mutually exclusive upper byte bits can be combined 
sim_navigation_passive                    =0x0000
sim_navigation_camerashift                =0x0001
sim_navigation_camerarotate                =0x0002
sim_navigation_camerazoom                =0x0003
sim_navigation_cameratilt                =0x0004
sim_navigation_cameraangle                =0x0005
sim_navigation_camerafly                =0x0006
sim_navigation_objectshift                =0x0007
sim_navigation_objectrotate                =0x0008
sim_navigation_reserved2                =0x0009
sim_navigation_reserved3                =0x000A
sim_navigation_jointpathtest            =0x000B
sim_navigation_ikmanip                    =0x000C
sim_navigation_objectmultipleselection    =0x000D
# Bit-combine following values and add them to one of above's values for a valid navigation mode 
sim_navigation_reserved4                =0x0100
sim_navigation_clickselection            =0x0200
sim_navigation_ctrlselection            =0x0400
sim_navigation_shiftselection            =0x0800
sim_navigation_camerazoomwheel            =0x1000
sim_navigation_camerarotaterightbutton    =0x2000



#Remote API constants
SIMX_VERSION                    =0 
# Remote API message header structure 
SIMX_HEADER_SIZE                =18
simx_headeroffset_crc           =0    # 1 simxUShort. Generated by the client or server. The CRC for the message 
simx_headeroffset_version       =2    # 1 byte. Generated by the client or server. The version of the remote API software 
simx_headeroffset_message_id    =3    # 1 simxInt. Generated by the client (and used in a reply by the server) 
simx_headeroffset_client_time   =7    # 1 simxInt. Client time stamp generated by the client (and sent back by the server) 
simx_headeroffset_server_time   =11    # 1 simxInt. Generated by the server when a reply is generated. The server timestamp 
simx_headeroffset_scene_id      =15    # 1 simxUShort. Generated by the server. A unique ID identifying the scene currently displayed 
simx_headeroffset_server_state  =17    # 1 byte. Generated by the server. Bit coded 0 set --> simulation not stopped 1 set --> simulation paused 2 set --> real-time switch on 3-5 edit mode type (0=no edit mode 1=triangle 2=vertex 3=edge 4=path 5=UI)  

# Remote API command header 
SIMX_SUBHEADER_SIZE                 =26
simx_cmdheaderoffset_mem_size       =0    # 1 simxInt. Generated by the client or server. The buffer size of the command. 
simx_cmdheaderoffset_full_mem_size  =4    # 1 simxInt. Generated by the client or server. The full buffer size of the command (applies to split chunks). 
simx_cmdheaderoffset_pdata_offset0  =8    # 1 simxUShort. Generated by the client or server. The amount of data that is part of the command identification. 
simx_cmdheaderoffset_pdata_offset1  =10    # 1 simxInt. Generated by the client or server. The amount of shift of the pure data buffer (applies to split chunks). 
simx_cmdheaderoffset_cmd=14    # 1 simxInt. Generated by the client (and used in a reply by the server). The command combined with the operation mode of the command. 
simx_cmdheaderoffset_delay_or_split =18    # 1 simxUShort. Generated by the client or server. The amount of delay in ms of a continuous command or the max. pure data size to send at once (applies to split commands). 
simx_cmdheaderoffset_sim_time       =20    # 1 simxInt. Generated by the server. The simulation time (in ms) when the command was executed (or 0 if simulation is not running) 
simx_cmdheaderoffset_status         =24    # 1 byte. Generated by the server. (1 bit 0 is set --> error in function execution on server side). The client writes bit 1 if command cannot be overwritten
simx_cmdheaderoffset_reserved       =25    # 1 byte. Not yet used 





# Regular operation modes 
simx_opmode_oneshot                =0x000000 # sends command as one chunk. Reply will also come as one chunk. Doesn't wait for the reply. 
simx_opmode_blocking            =0x010000 # sends command as one chunk. Reply will also come as one chunk. Waits for the reply (_REPLY_WAIT_TIMEOUT_IN_MS is the timeout). 
simx_opmode_oneshot_wait        =0x010000 # sends command as one chunk. Reply will also come as one chunk. Waits for the reply (_REPLY_WAIT_TIMEOUT_IN_MS is the timeout). 
simx_opmode_continuous            =0x020000  
simx_opmode_streaming            =0x020000 # sends command as one chunk. Command will be stored on the server and always executed 
                                                      #(every x ms (as far as possible) where x can be 0-65535. just add x to opmode_continuous). 
                                                      # A reply will be sent continuously each time as one chunk. Doesn't wait for the reply. 

# Operation modes for heavy data 
simx_opmode_oneshot_split        =0x030000   # sends command as several chunks (max chunk size is x bytes where x can be _MIN_SPLIT_AMOUNT_IN_BYTES-65535. Just add x to opmode_oneshot_split). Reply will also come as several chunks. Doesn't wait for the reply.      
simx_opmode_continuous_split    =0x040000
simx_opmode_streaming_split    =0x040000    # sends command as several chunks (max chunk size is x bytes where x can be _MIN_SPLIT_AMOUNT_IN_BYTES-65535. Just add x to opmode_continuous_split). Command will be stored on the server and always executed. A reply will be sent continuously each time as several chunks. Doesn't wait for the reply. 

# Special operation modes 
simx_opmode_discontinue            =0x050000    # removes and cancels all commands stored on the client or server side (also continuous commands) 
simx_opmode_buffer                =0x060000    # doesn't send anything but checks if a reply for the given command is available in the input buffer (i.e. previously received from the server) 
simx_opmode_remove                =0x070000    # doesn't send anything and doesn't return any specific value. It just erases a similar command reply in the inbox (to free some memory) 


# Command return codes 
simx_return_ok                    =0x000000
simx_return_novalue_flag        =0x000001        # input buffer doesn't contain the specified command 
simx_return_timeout_flag        =0x000002        # command reply not received in time for opmode_oneshot_wait operation mode 
simx_return_illegal_opmode_flag    =0x000004        # command doesn't support the specified operation mode 
simx_return_remote_error_flag    =0x000008        # command caused an error on the server side 
simx_return_split_progress_flag    =0x000010        # previous similar command not yet fully processed (applies to opmode_oneshot_split operation modes) 
simx_return_local_error_flag    =0x000020        # command caused an error on the client side 
simx_return_initialize_error_flag    =0x000040        # simxStart was not yet called 

# Following for backward compatibility (same as above) 
simx_error_noerror                =0x000000
simx_error_novalue_flag            =0x000001        # input buffer doesn't contain the specified command 
simx_error_timeout_flag            =0x000002        # command reply not received in time for opmode_oneshot_wait operation mode 
simx_error_illegal_opmode_flag    =0x000004        # command doesn't support the specified operation mode 
simx_error_remote_error_flag    =0x000008        # command caused an error on the server side 
simx_error_split_progress_flag    =0x000010        # previous similar command not yet fully processed (applies to opmode_oneshot_split operation modes) 
simx_error_local_error_flag        =0x000020        # command caused an error on the client side 
simx_error_initialize_error_flag        =0x000040        # simxStart was not yet called 


