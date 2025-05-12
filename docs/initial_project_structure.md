helios_emc/
│
├── model/
│   ├── __init__.py
│   ├── can_interface.py      # CAN bus communication
│   ├── data_storage.py       # Database/dictionary management
│   ├── signal_mapping.py     # DBC signal to data structure mapping
│   └── logger.py             # Data logging functionality
│
├── controller/
│   ├── __init__.py
│   ├── data_processor.py     # Process raw data
│   ├── app_controller.py     # Main application controller
│   └── config_manager.py     # Configuration management
│
├── view/
│   ├── __init__.py
│   ├── main_window.py        # Main application window
│   ├── table_view.py         # Tabular data visualization
│   ├── plot_view.py          # Real-time plots
│   └── components/           # Reusable UI components
│       ├── __init__.py
│       ├── counters.py       # Common counter components
│       └── buttons.py        # Common button components
│
├── resources/
│
├── utils/
│   ├── __init__.py
│   └── event_system.py       # Custom event handling
│
├── config/
│   ├── app_config.json       # Application configuration
│   └── can_mappings.json     # CAN signal to data mapping
│
├── .gitignore                # Git ignore file
├── pyproject.toml            # Project toml file
├── folder_paths.py           # Folder path management
├── setup.cfg                 # Project setup configuration
├── README.md                 # Project main documentation
└── requirements.txt          # Project dependencies

