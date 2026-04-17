import { Wheat, ChefHat, Coffee, CupSoda, Paperclip, Dog, Sparkles, ShoppingBasket } from "lucide-react";

const iconMap = {
  "Dry Goods & Grains": Wheat,
  "Baking & Canned": ChefHat,
  "Snacks & Breakfast": Coffee,
  "Beverages": CupSoda,
  "Office Supplies": Paperclip,
  "Pet Supplies": Dog,
  "Household & Personal Care": Sparkles,
  "All Products": ShoppingBasket,
};

export default function CategoryCards({ categories = [], onSelect }) {
  return (
    <div className="grid w-full grid-cols-2 sm:grid-cols-4 gap-8 max-w-full">
      {categories.map((cat) => {
        const Icon = iconMap[cat] || ShoppingBasket;
        return (
          <button
            key={cat}
            type="button"
            onClick={() => onSelect(cat)}
            className="rounded-xl bg-[#500000] text-white shadow-xl p-6 flex flex-col items-center justify-center h-[240px] w-full touch-manipulation focus:outline-none transition-colors hover:bg-[#610000]"
            aria-label={`Open ${cat} category`}
          >
            <div className="text-3xl font-bold mb-4">{cat}</div>
            <Icon size={64} strokeWidth={1.5} />
          </button>
        );
      })}
    </div>
  );
}