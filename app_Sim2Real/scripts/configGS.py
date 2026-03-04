
def get_args(args,scene, model):
    # This function sets the model path and other parameters based on the scene and model type.
    # It is used to configure the Gaussian Splatting model for rendering images.
    # Add new scenes here
    if scene == 'scene1':
        offset_GS = [0,0]
        camera_model = "/workspace/data/scientific_park_scene1/sparse/cameras.txt"
        if model == 'gof': base_path = "/workspace/outputs/scientific_park_scene1/gof/25_04_10_10_17/"
        elif model == "hierarchical3DGS": base_path = "/workspace/outputs/scientific_park_scene1/hierarchical/25_04_10_10_25/"
        elif model == "octreeGS": base_path = "/workspace/outputs/scientific_park_scene1/Octree-GS/25_04_30_06_57/"
    elif scene == 'scene2':
        offset_GS = [-90,70]
        camera_model = "/workspace/data/scientific_park_scene2/sparse/cameras.txt"
        if model == 'gof': base_path = "/workspace/outputs/scientific_park_scene2/gof/25_03_26_11_06"
        elif model == "hierarchical3DGS": base_path = "/workspace/outputs/scientific_park_scene2/hierarchical/25_04_03_10_27/"
        elif model == "octreeGS": base_path = "/workspace/outputs/scientific_park_scene2/Octree-GS/25_05_02_07_37/"
    elif scene == 'scene3':
        offset_GS = [22,-193]
        camera_model = "/workspace/data/scientific_park_scene3/sparse/cameras.txt"
        if model == 'gof': base_path = "/workspace/outputs/scientific_park_scene3/gof/25_04_14_11_46"
        elif model == "hierarchical3DGS": base_path = "/workspace/outputs/scientific_park_scene3/hierarchical/25_04_15_08_44/"
        elif model == "octreeGS": base_path = "/workspace/outputs/scientific_park_scene3/Octree-GS/25_05_02_09_06/"
    elif scene == 'scene4':
        offset_GS = [-174,-203]
        camera_model = "/workspace/data/scientific_park_scene4/sparse/cameras.txt"
        if model == 'gof': base_path = "/workspace/outputs/scientific_park_scene4/gof/25_05_06_07_40/"
        elif model == "hierarchical3DGS": base_path = "/workspace/outputs/scientific_park_scene4/hierarchical/25_05_06_10_01/"
        elif model == "octreeGS": base_path = "/workspace/outputs/scientific_park_scene4/Octree-GS/25_05_06_06_51/"
    elif scene == 'test_route1':
        offset_GS = [-105,-382]
        camera_model = "/workspace/data/test_route1/sparse/cameras.txt"
        if model == 'gof': base_path = "/workspace/outputs/test_route1/gof/25_05_19_06_49/"
        elif model == "hierarchical3DGS": base_path = "/workspace/outputs/test_route1/hierarchical/25_05_19_07_09/"
        elif model == "octreeGS": base_path = "/workspace/outputs/test_route1/Octree-GS/25_05_19_07_58/"
    elif scene == 'gof_scene1':
        offset_GS = [111,-346]
        camera_model = "/workspace/data/scientific_park_scene1/sparse/cameras.txt"
        if model == 'gof': base_path = "/workspace/outputs/scientific_park_scene1/gof/25_04_10_10_17/"
        elif model == "hierarchical3DGS": base_path = "/workspace/outputs/scientific_park_scene1/hierarchical/25_04_10_10_25/"
        elif model == "octreeGS": base_path = "/workspace/outputs/scientific_park_scene1/Octree-GS/25_04_30_06_57/"
    elif scene == 'gof_scene2':
        offset_GS = [44,-482]
        camera_model = "/workspace/data/scientific_park_scene2/sparse/cameras.txt"
        if model == 'gof': base_path = "/workspace/outputs/scientific_park_scene2/gof/25_03_26_11_06"
        elif model == "hierarchical3DGS": base_path = "/workspace/outputs/scientific_park_scene2/hierarchical/25_04_03_10_27/"
        elif model == "octreeGS": base_path = "/workspace/outputs/scientific_park_scene2/Octree-GS/25_05_02_07_37/"
    else:
        raise ValueError("Invalid scene name.")
    #Save the arguments
    args.offset_GS = offset_GS
    args.camera_model = camera_model
    if model == "gof" or model == "octreeGS":
        args.model_path = base_path
    elif model == "hierarchical3DGS":  
        args.model_path = base_path + "scaffold/"
        args.hierarchy = base_path + "merged.hier"
        args.scaffold_file = base_path + "scaffold/point_cloud/iteration_30000"
        args.transform_GS = base_path + "transform.json"
    else:
        raise ValueError("Invalid model name.")