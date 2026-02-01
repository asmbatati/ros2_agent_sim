import React, { useState } from 'react';
import { motion } from 'framer-motion';
import { Box, User, Zap, Play, Trash2, Cpu, Grid } from 'lucide-react';

// Item Types
const ITEM_TYPES = {
  ROBOT: 'robot',
  ASSET: 'asset',
  AGENT: 'agent'
};

// Available Items
const AVAILABLE_ITEMS = [
  { id: 'go2', name: 'Unitree Go2', type: ITEM_TYPES.ROBOT, icon: Zap },
  { id: 'drone', name: 'x500 Drone', type: ITEM_TYPES.ROBOT, icon: Zap },
  { id: 'person', name: 'Person', type: ITEM_TYPES.ASSET, icon: User },
  { id: 'box', name: 'Crate', type: ITEM_TYPES.ASSET, icon: Box },
  { id: 'rosa', name: 'ROSA Agent', type: ITEM_TYPES.AGENT, icon: Cpu },
];

function App() {
  const [placedItems, setPlacedItems] = useState([]);
  const [draggedItem, setDraggedItem] = useState(null);

  const handleDragStart = (e, item) => {
    setDraggedItem(item);
    // Note: JSON.stringify will strip out the 'icon' function property
    e.dataTransfer.setData('item', JSON.stringify(item));
  };

  const handleDrop = (e) => {
    e.preventDefault();
    const itemData = e.dataTransfer.getData('item');
    if (!itemData) return;

    const item = JSON.parse(itemData);
    const rect = e.currentTarget.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;

    // Convert pixels to relative coordinates
    const gridWidth = rect.width;
    const gridHeight = rect.height;

    // Map to roughly -10 to 10 meters
    const worldX = ((x / gridWidth) * 20 - 10).toFixed(2);
    const worldY = -((y / gridHeight) * 20 - 10).toFixed(2);

    const newItem = {
      ...item,
      instanceId: Date.now(),
      x: parseFloat(worldX),
      y: parseFloat(worldY),
      pixelX: x,
      pixelY: y
    };

    setPlacedItems([...placedItems, newItem]);
    setDraggedItem(null);
  };

  const handleDragOver = (e) => {
    e.preventDefault();
  };

  const removeItem = (id) => {
    setPlacedItems(placedItems.filter(item => item.instanceId !== id));
  };

  const launchSimulation = async () => {
    console.log("Launching simulation with config:", placedItems);
    try {
      const response = await fetch('/launch', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ items: placedItems })
      });
      const data = await response.json();
      alert(`Simulation Launched! ID: ${data.launch_id}`);
    } catch (error) {
      console.error("Failed to launch:", error);
      alert("Failed to launch simulation. Is the backend running?");
    }
  };

  // Helper to get icon component
  const getIcon = (id) => {
    const found = AVAILABLE_ITEMS.find(i => i.id === id);
    return found ? found.icon : Box;
  };

  return (
    <div className="flex h-screen bg-slate-900 text-white font-sans overflow-hidden">
      {/* Sidebar */}
      <div className="w-80 bg-slate-800 border-r border-slate-700 p-6 flex flex-col shadow-xl z-10">
        <h1 className="text-2xl font-bold mb-6 bg-gradient-to-r from-blue-400 to-purple-500 bg-clip-text text-transparent">
          Sim Setup
        </h1>

        <div className="space-y-6 flex-1 overflow-y-auto">
          {/* Robots Section */}
          <div>
            <h2 className="text-sm uppercase tracking-wider text-slate-400 mb-3 font-semibold">Robots</h2>
            <div className="grid grid-cols-2 gap-3">
              {AVAILABLE_ITEMS.filter(i => i.type === ITEM_TYPES.ROBOT).map(item => (
                <DraggableCard key={item.id} item={item} onDragStart={handleDragStart} />
              ))}
            </div>
          </div>

          {/* Assets Section */}
          <div>
            <h2 className="text-sm uppercase tracking-wider text-slate-400 mb-3 font-semibold">Environment</h2>
            <div className="grid grid-cols-2 gap-3">
              {AVAILABLE_ITEMS.filter(i => i.type === ITEM_TYPES.ASSET).map(item => (
                <DraggableCard key={item.id} item={item} onDragStart={handleDragStart} />
              ))}
            </div>
          </div>

          {/* Agents Section */}
          <div>
            <h2 className="text-sm uppercase tracking-wider text-slate-400 mb-3 font-semibold">Agents</h2>
            <div className="grid grid-cols-1 gap-3">
              {AVAILABLE_ITEMS.filter(i => i.type === ITEM_TYPES.AGENT).map(item => (
                <DraggableCard key={item.id} item={item} onDragStart={handleDragStart} fullWidth />
              ))}
            </div>
          </div>
        </div>

        <button
          onClick={launchSimulation}
          className="mt-6 w-full py-4 bg-gradient-to-r from-green-500 to-emerald-600 hover:from-green-600 hover:to-emerald-700 rounded-xl font-bold shadow-lg transform transition active:scale-95 flex items-center justify-center gap-2"
        >
          <Play size={20} />
          Launch Simulation
        </button>
      </div>

      {/* Main Content / Drop Zone */}
      <div className="flex-1 relative bg-slate-900 overflow-hidden">
        {/* Grid Background */}
        <div
          className="absolute inset-0 opacity-20 pointer-events-none"
          style={{
            backgroundImage: 'linear-gradient(#334155 1px, transparent 1px), linear-gradient(90deg, #334155 1px, transparent 1px)',
            backgroundSize: '40px 40px'
          }}
        />

        {/* Center Marker */}
        <div className="absolute left-1/2 top-1/2 w-4 h-4 bg-blue-500 rounded-full opacity-50 transform -translate-x-1/2 -translate-y-1/2 pointer-events-none" />

        <div
          className="w-full h-full relative"
          onDrop={handleDrop}
          onDragOver={handleDragOver}
        >
          <div className="absolute top-4 left-4 bg-slate-800/80 backdrop-blur px-4 py-2 rounded-lg border border-slate-700 text-sm text-slate-300">
            Drag items here to place them in the world
          </div>

          {placedItems.map((item) => {
            const Icon = getIcon(item.id);
            return (
              <motion.div
                key={item.instanceId}
                initial={{ scale: 0, opacity: 0 }}
                animate={{ scale: 1, opacity: 1 }}
                className="absolute group cursor-grab active:cursor-grabbing"
                style={{
                  left: item.pixelX,
                  top: item.pixelY,
                  transform: 'translate(-50%, -50%)'
                }}
              >
                <div className={`
                  w-16 h-16 rounded-xl shadow-2xl flex items-center justify-center relative
                  ${item.type === ITEM_TYPES.ROBOT ? 'bg-blue-600' :
                    item.type === ITEM_TYPES.AGENT ? 'bg-purple-600' : 'bg-slate-600'}
                `}>
                  <Icon size={32} className="text-white" />

                  {/* Remove Button */}
                  <button
                    onClick={(e) => { e.stopPropagation(); removeItem(item.instanceId); }}
                    className="absolute -top-2 -right-2 bg-red-500 text-white p-1 rounded-full opacity-0 group-hover:opacity-100 transition-opacity shadow-md hover:bg-red-600"
                  >
                    <Trash2 size={12} />
                  </button>
                </div>
                <div className="absolute top-full mt-2 left-1/2 transform -translate-x-1/2 bg-black/70 px-2 py-1 rounded text-xs whitespace-nowrap">
                  {item.name} <br /> ({item.x}m, {item.y}m)
                </div>
              </motion.div>
            );
          })}
        </div>
      </div>
    </div>
  );
}

function DraggableCard({ item, onDragStart, fullWidth }) {
  return (
    <div
      draggable
      onDragStart={(e) => onDragStart(e, item)}
      className={`
        bg-slate-700 hover:bg-slate-600 border border-slate-600 hover:border-blue-500 
        transition-all duration-200 cursor-grab active:cursor-grabbing 
        p-3 rounded-lg shadow-sm group flex flex-col items-center gap-2
        ${fullWidth ? 'flex-row px-4' : ''}
      `}
    >
      <div className={`
        p-2 rounded-lg bg-slate-800 group-hover:bg-slate-700 transition-colors
        ${item.type === ITEM_TYPES.ROBOT ? 'text-blue-400' :
          item.type === ITEM_TYPES.AGENT ? 'text-purple-400' : 'text-slate-400'}
      `}>
        <item.icon size={24} />
      </div>
      <span className="text-xs font-medium text-slate-300 group-hover:text-white text-center">
        {item.name}
      </span>
    </div>
  );
}

export default App;
