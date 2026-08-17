class InvalidYamlContentError(Exception):
    pass

def validate_yaml(building_yaml, config_yaml, level: str):
    validate_building_yaml(building_yaml, level)
    validate_config_yaml(config_yaml, level)

    print(f'Checking if building scale obtainable...')
    has_measurements = bool(building_yaml['levels'][level].get('measurements'))
    has_transform_scale = (
        config_yaml['map_transform'][level]
        .get('level_transform', {})
        .get('scale') is not None)
    if not has_measurements and has_transform_scale:
        print(
            'WARNING: No measurements found in building.yaml; using '
            'config level_transform.scale for building scale.')
    elif not has_measurements and not has_transform_scale:
        raise InvalidYamlContentError(
            'Cannot obtain building scale: add measurements to building.yaml '
            'or set map_transform.' + level + '.level_transform.scale in '
            'config.yaml.')

def validate_building_yaml(building_yaml, level: str):
    print(f'Validating building.yaml file')
    if 'levels' not in building_yaml or level not in building_yaml['levels']:
        raise InvalidYamlContentError(f'Level {level} does not exist in building.yaml file!')
    
    if 'name' not in building_yaml:
        raise InvalidYamlContentError(f'building.yaml file does not have a building name!')

def validate_config_yaml(config_yaml, level: str):
    def transform_config_malformed(config_file: dict, level: str):
      print(f'Validating map transforms in config.yaml file')
      if 'map_transform' not in config_file or level not in config_file['map_transform']:
          return True
      
      map_transform = config_file['map_transform'][level]
      
      if 'transform_values' in map_transform:
          expected_keys = {'tx_meters', 'ty_meters', 'rotation_degrees', 'scale'}
          return set(map_transform['transform_values'].keys()) != expected_keys
      else: 
          return True
      
    print(f'Validating config.yaml file')
    if 'rmf_fleet' not in config_yaml or 'name' not in config_yaml['rmf_fleet']:
        raise InvalidYamlContentError(f'config.yaml file does not contain fleet name!')
    
    if transform_config_malformed(config_yaml, level):
        raise InvalidYamlContentError(f'Transforms for level {level} not in config.yaml file!')
