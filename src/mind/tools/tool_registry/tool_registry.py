import inspect
from typing import Any, Dict, Optional, Union, Callable
from mind.tools import Timer

class ToolRegistry:
    def __init__(self):
        self._registry = {}

    def register(self, name_or_func: Union[str, Callable] = None):
        """
        Robust decorator. Works as both @register and @register(name="foo").
        """
        # Case 1: Called as @register
        if callable(name_or_func):
            func = name_or_func
            self._registry[func.__name__] = func
            return func

        # Case 2: Called as @register(name="func") or @register()
        def decorator(obj):
            tool_name = name_or_func if isinstance(name_or_func, str) else obj.__name__
            self._registry[tool_name] = obj
            return obj
        return decorator
    
    # ... inside ToolRegistry class ...

    def add_tool(self, tool: Any, name: str = None):
        """
        Manually register a tool without using a decorator.
        Useful when importing tools from other files.
        """
        if not callable(tool):
             raise ValueError(f"Tool must be a class or function, got {type(tool)}")

        # Use provided name, or fallback to the tool's original name
        tool_name = name if name else tool.__name__
        self._registry[tool_name] = tool
        print(f"✅ Registered '{tool_name}' manually.")

    def get_instance(self, name: str, *args, **kwargs) -> Any:
        """
        If Class -> Returns Instance (Factory).
        If Function -> Returns None (as per your specific request).
        """
        obj = self._registry.get(name)
        if not obj:
            raise KeyError(f"Tool '{name}' not found.")
        
        if inspect.isclass(obj):
            # Factory: Create the object
            return obj(*args, **kwargs)
        
        # Requirement: "if it is just a function nothing"
        return None 

    def get_details(self, target: Union[str, Any], detailed: bool = False) -> Dict[str, Any]:
        """
        Master Lookup: Handles Names (str), Classes, Functions, and Instances.
        """
        # 1. Resolve Name -> Object
        if isinstance(target, str):
            if target not in self._registry:
                raise KeyError(f"Tool '{target}' not found.")
            target = self._registry[target]

        meta = {}
        
        # --- CASE A: Function ---
        if inspect.isroutine(target): 
            meta = {
                "kind": "function",
                "name": target.__name__,
                "doc": inspect.getdoc(target),
                "signature": str(inspect.signature(target))
            }

        # --- CASE B: Class (Factory) ---
        elif inspect.isclass(target):
            meta = {
                "kind": "class",
                "name": target.__name__,
                "doc": inspect.getdoc(target),
                "signature": self._safe_signature(target) # Shows __init__ sig
            }
            if detailed:
                meta["methods"] = self._scan_methods(target)

        # --- CASE C: Instance ---
        else:
            meta = {
                "kind": "instance",
                "class_name": type(target).__name__,
                "doc": inspect.getdoc(type(target)),
            }
            if detailed:
                meta["methods"] = self._scan_methods(target)

        return meta

    def _scan_methods(self, obj) -> Dict[str, Any]:
        """Scans public methods and returns a Dictionary for easier lookup."""
        methods = {}
        for attr_name in dir(obj):
            if attr_name.startswith("_"): continue 
            
            try:
                attr_value = getattr(obj, attr_name)
                if callable(attr_value):
                    methods[attr_name] = {
                        "doc": inspect.getdoc(attr_value),
                        "signature": self._safe_signature(attr_value)
                    }
            except Exception:
                continue 
        return methods

    def _safe_signature(self, obj):
        """Prevents crashes on weird objects."""
        try:
            return str(inspect.signature(obj))
        except ValueError:
            return "(...)"

# --- USAGE DEMO ---

registry = ToolRegistry()

# Works without parens now!
registry.add_tool(Timer, name="MyTimer")

# Works with custom name!
registry.add_tool(Timer.start, name="start_timer")

# 1. List and Get Details
print(registry.get_details("MyTimer", detailed=True))

# 2. Get Instance
# This works for Class
my_tool = registry.get_instance("MyTimer") 
print(f"/n/nGot instance: {my_tool}")

# This returns None (as requested) because 'quick_calc' is a function
func_result = registry.get_instance("start_timer")
print(f"/n/nFunction instance result: {func_result}")

# 3. Instance Details
# Now we inspect the instance we just made
print(registry.get_details("start_timer", detailed=True))